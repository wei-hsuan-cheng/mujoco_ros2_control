// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"

#include <cmath>

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{
MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data)
    : node_(node),
      mj_model_(mujoco_model),
      mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
{
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();

  if (cm_thread_.joinable()) cm_thread_.join();
}

std::string MujocoRos2Control::get_robot_description()
{
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
    "robot_description_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (node->has_parameter("robot_description"))
  {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
    logger_,
    "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
    "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
    });

  while (robot_description.empty() && rclcpp::ok())
  {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}

void MujocoRos2Control::init()
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  std::string urdf_string = this->get_robot_description();

  // setup actuators and mechanism control node.
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try
  {
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  }
  catch (const std::runtime_error &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

  try
  {
    robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<MujocoSystemInterface>(
      "mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
  }
  catch (pluginlib::LibraryLoadException &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<hardware_interface::ResourceManager>();

  try
  {
    resource_manager->load_urdf(urdf_string, false, false);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Error while initializing URDF!");
  }

  for (const auto &hardware : control_hardware_info)
  {
    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try
    {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    }
    catch (pluginlib::PluginlibException &ex)
    {
      RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
      continue;
    }

    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    if (!mujoco_system->init_sim(mj_model_, mj_data_, urdf_model, hardware))
    {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return;
    }

    resource_manager->import_component(std::move(mujoco_system), hardware);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace());
  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate"))
  {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  init_ground_truth();
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);
  if (gt_enabled_)
  {
    publish_ground_truth(sim_time_ros);
  }

  mj_step1(mj_model_, mj_data_);

  if (sim_period >= control_period_)
  {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  mj_step2(mj_model_, mj_data_);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // TODO(sangteak601)
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

void MujocoRos2Control::init_ground_truth()
{
  if (!node_->has_parameter("gt_enabled"))
    node_->declare_parameter<bool>("gt_enabled", false);
  gt_enabled_ = node_->get_parameter("gt_enabled").as_bool();
  if (!gt_enabled_) return;

  if (!node_->has_parameter("gt_publish_tf"))
    node_->declare_parameter<bool>("gt_publish_tf", false);
  if (!node_->has_parameter("gt_pub_hz"))
    node_->declare_parameter<double>("gt_pub_hz", 120.0);
  if (!node_->has_parameter("gt_odom_topic"))
    node_->declare_parameter<std::string>("gt_odom_topic", "/mujoco/ground_truth/odom");
  if (!node_->has_parameter("gt_root_frame"))
    node_->declare_parameter<std::string>("gt_root_frame", "world");
  if (!node_->has_parameter("gt_frame_prefix"))
    node_->declare_parameter<std::string>("gt_frame_prefix", "");
  if (!node_->has_parameter("gt_frame_suffix"))
    node_->declare_parameter<std::string>("gt_frame_suffix", "");
  if (!node_->has_parameter("gt_body_frames"))
    node_->declare_parameter<std::vector<std::string>>("gt_body_frames", std::vector<std::string>{});

  gt_publish_tf_   = node_->get_parameter("gt_publish_tf").as_bool();
  gt_pub_hz_       = node_->get_parameter("gt_pub_hz").as_double();
  gt_odom_topic_   = node_->get_parameter("gt_odom_topic").as_string();
  gt_root_frame_   = node_->get_parameter("gt_root_frame").as_string();
  gt_frame_prefix_ = node_->get_parameter("gt_frame_prefix").as_string();
  gt_frame_suffix_ = node_->get_parameter("gt_frame_suffix").as_string();

  auto requested_body_frames = node_->get_parameter("gt_body_frames").as_string_array();

  // ---------------- PROTECT: if not specified, fallback ----------------
  if (requested_body_frames.empty())
  {
    // Try a sensible default for your robot
    requested_body_frames = std::vector<std::string>{"mobile_base_link"};

    RCLCPP_WARN_STREAM(
      logger_,
      "Ground truth enabled but parameter 'gt_body_frames' is empty. "
      "Falling back to default: [mobile_base_link]. "
      "If you want more frames, set 'gt_body_frames' in your YAML.");
  }

  // Period
  if (gt_pub_hz_ > 0.0 && std::isfinite(gt_pub_hz_))
    gt_period_ = rclcpp::Duration::from_seconds(1.0 / gt_pub_hz_);
  else
    gt_period_ = rclcpp::Duration(0, 0);

  // TF broadcaster / odom pub
  if (gt_publish_tf_)
    gt_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  if (!gt_odom_topic_.empty())
    gt_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(gt_odom_topic_, rclcpp::QoS(10));

  // Resolve body IDs
  gt_body_ids_.clear();
  gt_body_frames_.clear();
  gt_body_ids_.reserve(requested_body_frames.size());
  gt_body_frames_.reserve(requested_body_frames.size());

  for (const auto &frame_name : requested_body_frames)
  {
    const int body_id = mj_name2id(mj_model_, mjOBJ_BODY, frame_name.c_str());
    if (body_id < 0)
    {
      RCLCPP_WARN_STREAM(
        logger_, "Ground truth: body '" << frame_name << "' not found in MuJoCo model; skipping.");
      continue;
    }
    gt_body_frames_.push_back(frame_name);
    gt_body_ids_.push_back(body_id);
  }

  // ------------- PROTECT: if still none valid, disable GT safely -------------
  if (gt_body_frames_.empty())
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Ground truth enabled but none of the requested bodies exist in the MuJoCo model. "
      "Disabling ground truth publishing.");
    gt_enabled_ = false;
    gt_tf_broadcaster_.reset();
    gt_odom_pub_.reset();
    return;
  }

  // Choose odom body: prefer mobile_base_link, else first valid
  gt_odom_child_frame_.clear();
  gt_odom_body_id_ = -1;

  for (size_t i = 0; i < gt_body_frames_.size(); ++i)
  {
    if (gt_body_frames_[i] == "mobile_base_link")
    {
      gt_odom_child_frame_ = gt_body_frames_[i];
      gt_odom_body_id_ = gt_body_ids_[i];
      break;
    }
  }
  if (gt_odom_body_id_ < 0)
  {
    gt_odom_child_frame_ = gt_body_frames_.front();
    gt_odom_body_id_ = gt_body_ids_.front();
    RCLCPP_WARN_STREAM(
      logger_,
      "Ground truth: 'mobile_base_link' not in gt_body_frames, using first valid body '"
        << gt_odom_child_frame_ << "' for odom.");
  }

  RCLCPP_INFO_STREAM(
    logger_,
    "Ground truth enabled (publish_tf=" << (gt_publish_tf_ ? "true" : "false")
    << ", hz=" << gt_pub_hz_
    << ", parent_frame=" << gt_root_frame_
    << ", odom_topic=" << gt_odom_topic_
    << ", odom_child_frame=" << gt_odom_child_frame_
    << ", prefix=" << gt_frame_prefix_
    << ", suffix=" << gt_frame_suffix_
    << ", bodies=" << gt_body_frames_.size() << ")");
}

void MujocoRos2Control::publish_ground_truth(const rclcpp::Time &stamp)
{
  if (!gt_enabled_)
  {
    return;
  }

  if (gt_period_ > rclcpp::Duration(0, 0) && (stamp - last_gt_pub_time_) < gt_period_)
  {
    return;
  }
  last_gt_pub_time_ = stamp;

  if (gt_publish_tf_ && gt_tf_broadcaster_)
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(gt_body_ids_.size());

    for (size_t i = 0; i < gt_body_ids_.size(); ++i)
    {
      const int body_id = gt_body_ids_[i];
      const auto &body_name = gt_body_frames_[i];
      const std::string child_frame = gt_frame_prefix_ + body_name + gt_frame_suffix_;

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = gt_root_frame_;
      tf_msg.child_frame_id = child_frame;

      tf_msg.transform.translation.x = mj_data_->xpos[3 * body_id + 0];
      tf_msg.transform.translation.y = mj_data_->xpos[3 * body_id + 1];
      tf_msg.transform.translation.z = mj_data_->xpos[3 * body_id + 2];

      // MuJoCo uses [w, x, y, z]; ROS uses [x, y, z, w]
      tf_msg.transform.rotation.w = mj_data_->xquat[4 * body_id + 0];
      tf_msg.transform.rotation.x = mj_data_->xquat[4 * body_id + 1];
      tf_msg.transform.rotation.y = mj_data_->xquat[4 * body_id + 2];
      tf_msg.transform.rotation.z = mj_data_->xquat[4 * body_id + 3];

      transforms.push_back(std::move(tf_msg));
    }

    gt_tf_broadcaster_->sendTransform(transforms);
  }

  if (gt_odom_pub_ && gt_odom_body_id_ >= 0 && !gt_odom_child_frame_.empty())
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = gt_root_frame_;
    odom.child_frame_id = gt_frame_prefix_ + gt_odom_child_frame_ + gt_frame_suffix_;

    const int body_id = gt_odom_body_id_;

    odom.pose.pose.position.x = mj_data_->xpos[3 * body_id + 0];
    odom.pose.pose.position.y = mj_data_->xpos[3 * body_id + 1];
    odom.pose.pose.position.z = mj_data_->xpos[3 * body_id + 2];

    odom.pose.pose.orientation.w = mj_data_->xquat[4 * body_id + 0];
    odom.pose.pose.orientation.x = mj_data_->xquat[4 * body_id + 1];
    odom.pose.pose.orientation.y = mj_data_->xquat[4 * body_id + 2];
    odom.pose.pose.orientation.z = mj_data_->xquat[4 * body_id + 3];

    mjtNum vel6[6] = {0};
    // Returns 6D velocity as (angular, linear) about object center.
    // flg_local=1 expresses vectors in the body's local frame, matching Odometry child frame.
    mj_objectVelocity(mj_model_, mj_data_, mjOBJ_BODY, body_id, vel6, /*flg_local=*/1);

    odom.twist.twist.angular.x = vel6[0];
    odom.twist.twist.angular.y = vel6[1];
    odom.twist.twist.angular.z = vel6[2];
    odom.twist.twist.linear.x = vel6[3];
    odom.twist.twist.linear.y = vel6[4];
    odom.twist.twist.linear.z = vel6[5];

    gt_odom_pub_->publish(odom);
  }
}

}  // namespace mujoco_ros2_control
