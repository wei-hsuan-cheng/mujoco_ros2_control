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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "mujoco_ros2_control/msg/mujoco_external_wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "mujoco/mujoco.h"

#include "mujoco_ros2_control/mujoco_system.hpp"

namespace mujoco_ros2_control
{
class MujocoRos2Control
{
public:
  MujocoRos2Control(rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data);
  ~MujocoRos2Control();
  void init();
  void update();
  void update_controller_manager();

private:
  struct PendingExternalWrench
  {
    std::string body_name;
    std::string wrench_frame;
    std::array<mjtNum, 6> wrench{};
  };

  struct ActiveExternalWrench
  {
    std::string body_name;
    std::string wrench_frame;
    std::array<mjtNum, 6> wrench{};
    double end_time{0.0};
  };

  void publish_sim_time(rclcpp::Time sim_time);
  void init_external_wrench();
  void external_wrench_callback(const mujoco_ros2_control::msg::MujocoExternalWrench::SharedPtr msg);
  void legacy_external_wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void queue_external_wrench_command(
    const std::string &body_name,
    const std::string &wrench_frame,
    const geometry_msgs::msg::Wrench &wrench);
  std::string normalize_external_wrench_frame(
    const std::string &wrench_frame,
    const std::string &body_name) const;
  bool transform_external_wrench_to_world(
    const ActiveExternalWrench &command,
    int target_body_id,
    std::array<mjtNum, 6> &world_wrench) const;
  void transform_world_wrench_to_body(
    int body_id,
    const std::array<mjtNum, 6> &world_wrench,
    std::array<mjtNum, 6> &body_wrench) const;
  void publish_external_wrench_visualization(
    const std::string &body_name,
    const std::array<mjtNum, 6> &body_wrench);
  void apply_external_wrenches();
  void init_ground_truth();
  void publish_ground_truth(const rclcpp::Time &stamp);
  std::string get_robot_description();
  rclcpp::Node::SharedPtr node_;
  mjModel *mj_model_;
  mjData *mj_data_;

  rclcpp::Logger logger_;
  std::shared_ptr<pluginlib::ClassLoader<MujocoSystemInterface>> robot_hw_sim_loader_;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  rclcpp::Executor::SharedPtr cm_executor_;
  std::thread cm_thread_;
  bool stop_cm_thread_;
  rclcpp::Duration control_period_;

  rclcpp::Time last_update_sim_time_ros_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  bool external_wrench_enabled_{true};
  double external_wrench_timeout_{0.1};
  std::string legacy_external_wrench_frame_{"world"};
  rclcpp::Subscription<mujoco_ros2_control::msg::MujocoExternalWrench>::SharedPtr external_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr legacy_external_wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_visualization_pub_;
  std::mutex external_wrench_mutex_;
  std::vector<PendingExternalWrench> pending_external_wrenches_;
  std::unordered_map<int, ActiveExternalWrench> active_external_wrenches_;

  bool gt_enabled_{false};
  bool gt_publish_tf_{false};
  double gt_pub_hz_{120.0};
  rclcpp::Duration gt_period_{0, 0};
  rclcpp::Time last_gt_pub_time_{0, 0, RCL_ROS_TIME};
  std::string gt_odom_topic_;
  std::string gt_root_frame_;
  std::string gt_frame_prefix_;
  std::string gt_frame_suffix_;
  std::vector<std::string> gt_body_frames_;
  std::vector<int> gt_body_ids_;
  int gt_odom_body_id_{-1};
  std::string gt_odom_child_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> gt_tf_broadcaster_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
