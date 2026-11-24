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

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <thread>

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mujoco_ros2_control/model_error.hpp"
#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();
  bool mujoco_headless = false;
  node->get_parameter_or("mujoco_headless", mujoco_headless, false);
  bool mujoco_wait_to_start = false;
  node->get_parameter_or("mujoco_wait_to_start", mujoco_wait_to_start, false);
  double mujoco_real_time_factor = 1.0;
  node->get_parameter_or("mujoco_real_time_factor", mujoco_real_time_factor, 1.0);
  if (
    !std::isfinite(mujoco_real_time_factor) ||
    mujoco_real_time_factor <= 0.0)
  {
    RCLCPP_FATAL(
      node->get_logger(),
      "mujoco_real_time_factor must be finite and greater than 0.0, got %.6f",
      mujoco_real_time_factor);
    rclcpp::shutdown();
    return 1;
  }

  // Plant-side modeling error (robustness studies): a YAML file perturbs the model in memory (ground patches are
  // added while loading, the other errors are applied to the compiled model).
  std::string model_error_file, model_error_report;
  int64_t model_error_seed = -1;
  node->get_parameter_or("model_error_file", model_error_file, std::string());
  node->get_parameter_or("model_error_seed", model_error_seed, int64_t{-1});
  node->get_parameter_or("model_error_report", model_error_report, std::string());

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (!model_error_file.empty()) {
    try {
      mujoco_model = mujoco_ros2_control::load_model(model_path, model_error_file);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(node->get_logger(), "model error: %s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  }
  else if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");

  if (!model_error_file.empty()) {
    try {
      std::string summary;
      const auto report =
        mujoco_ros2_control::apply_model_error(mujoco_model, model_error_file, model_error_seed, summary);
      RCLCPP_WARN_STREAM(node->get_logger(), summary);
      if (!model_error_report.empty()) {
        std::ofstream(model_error_report) << report << '\n';
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(node->get_logger(), "model error: %s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  }

  // make data
  mujoco_data = mj_makeData(mujoco_model);
  if (!model_error_file.empty()) {
    mj_setConst(mujoco_model, mujoco_data);  // derived constants (subtree masses, inverse weights) of the new model
  }

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  std::atomic_bool simulation_started{!mujoco_wait_to_start};
  auto start_service = node->create_service<std_srvs::srv::Trigger>(
    "~/start",
    [&simulation_started](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const bool was_started = simulation_started.exchange(true);
      response->success = true;
      response->message = was_started ? "MuJoCo is already running" : "MuJoCo simulation started";
    });

  auto node_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  node_executor->add_node(node);
  std::thread node_thread([node_executor]() { node_executor->spin(); });

  if (mujoco_wait_to_start) {
    RCLCPP_INFO(
      node->get_logger(),
      "MuJoCo physics is waiting for /mujoco_ros2_control/start");
    while (rclcpp::ok() && !simulation_started.load()) {
      // Complete controller lifecycle switches and stage commands without advancing physics.
      mujoco_control.update_controller_manager();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  using SteadyClock = std::chrono::steady_clock;
  constexpr double pacing_interval = 0.001;
  const auto wall_time_start = SteadyClock::now();
  const mjtNum simulation_time_start = mujoco_data->time;
  mjtNum next_pacing_time = simulation_time_start + pacing_interval;

  auto update_simulation = [&]() {
    mujoco_control.update();
    if (mujoco_data->time < next_pacing_time)
    {
      return;
    }

    const auto elapsed_simulation_time =
      std::chrono::duration<double>(
        (mujoco_data->time - simulation_time_start) / mujoco_real_time_factor);
    const auto target_wall_time = wall_time_start +
      std::chrono::duration_cast<SteadyClock::duration>(elapsed_simulation_time);
    std::this_thread::sleep_until(target_wall_time);
    next_pacing_time = mujoco_data->time + pacing_interval;
  };

  RCLCPP_INFO(
    node->get_logger(), "MuJoCo real-time pacing enabled (target RTF: %.3f)",
    mujoco_real_time_factor);

  if (mujoco_headless)
  {
    RCLCPP_INFO(node->get_logger(), "Running MuJoCo in headless mode");
    while (rclcpp::ok())
    {
      update_simulation();
    }
  }
  else
  {
    // initialize mujoco visualization environment for rendering and cameras
    if (!glfwInit())
    {
      mju_error("Could not initialize GLFW");
    }
    auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
    rendering->init(mujoco_model, mujoco_data);
    rendering->configure_camera_from_ros_params(node);
    RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

    auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
    cameras->init(mujoco_model);

    // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
    mjtNum last_cam_update = mujoco_data->time;
    while (rclcpp::ok() && !rendering->is_close_flag_raised())
    {
      // Advance simulation by 1/60 second between rendered frames.
      mjtNum simstart = mujoco_data->time;
      while (mujoco_data->time - simstart < 1.0 / 60.0)
      {
        update_simulation();
      }
      rendering->update();

      // Updating cameras at ~6 Hz
      // TODO(eholum): Break control and rendering into separate processes
      if (simstart - last_cam_update > 1.0 / 6.0)
      {
        cameras->update(mujoco_model, mujoco_data);
        last_cam_update = simstart;
      }
    }

    rendering->close();
    cameras->close();
  }

  node_executor->cancel();
  if (node_thread.joinable()) {
    node_thread.join();
  }

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}
