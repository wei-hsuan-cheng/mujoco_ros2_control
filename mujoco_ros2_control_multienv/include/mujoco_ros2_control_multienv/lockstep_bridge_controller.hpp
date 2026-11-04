// Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

#ifndef MUJOCO_ROS2_CONTROL_MULTIENV__LOCKSTEP_BRIDGE_CONTROLLER_HPP_
#define MUJOCO_ROS2_CONTROL_MULTIENV__LOCKSTEP_BRIDGE_CONTROLLER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "mujoco_ros2_control_multienv/lockstep_protocol.hpp"

namespace mujoco_ros2_control_multienv
{

/**
 * Steps a ros2_control simulation in lockstep with an external process (an RL trainer)
 * through shared memory, with no topic in between.
 *
 * Every step_period of controller time it copies the state interfaces it claims (any
 * hardware, broadcaster or chained-controller state interface) into the observation
 * block, and then blocks the controller manager - and with it the simulation that calls it,
 * e.g. mujoco_ros2_control - until the client answers with the values for the command
 * interfaces it claims (typically another controller's reference interfaces). Between
 * steps it keeps writing the last answer. So the client sees every step at exactly the
 * same controller time, and its answer is in effect from the same tick on, whatever the
 * load of the machine: the latency is fixed, not measured.
 *
 * Claiming a controller's reference interfaces and also reading that controller's exported
 * state makes the controller manager's update order between the two ambiguous. order_probe
 * names a state interface holding that controller's latest update time: if it already
 * equals this tick's time the controller ran first, and the exchange moves one tick earlier
 * so the answer still reaches it on the step tick (handshake_tick_offset -1 in the header).
 */
class LockstepBridgeController : public controller_interface::ControllerInterface
{
public:
  ~LockstepBridgeController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool map_shared_memory();
  void unmap_shared_memory();
  //! Observations out, wait, actions in. False when shutting down while waiting.
  bool exchange(double time);
  void write_commands();

  std::string shm_name_;
  double step_period_{0.01};
  std::vector<std::string> observed_interfaces_;
  std::vector<std::string> commanded_interfaces_;
  std::vector<double> command_defaults_;
  std::string order_probe_;
  double wait_log_period_{5.0};

  int64_t step_ticks_{1};
  int64_t handshake_offset_{0};
  bool order_known_{false};
  std::size_t probe_index_{0};  //!< in state_interfaces_: an observed one, or the extra one claimed last
  uint64_t step_{0};
  std::vector<double> commands_;

  int shm_fd_{-1};
  std::size_t shm_bytes_{0};
  void * shm_{nullptr};
  lockstep::Header * header_{nullptr};
  double * obs_{nullptr};
  double * act_{nullptr};
  bool semaphores_initialized_{false};
};

}  // namespace mujoco_ros2_control_multienv

#endif  // MUJOCO_ROS2_CONTROL_MULTIENV__LOCKSTEP_BRIDGE_CONTROLLER_HPP_
