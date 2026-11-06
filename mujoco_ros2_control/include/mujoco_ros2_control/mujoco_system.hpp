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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "mujoco_ros2_control/mujoco_lidar.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

namespace mujoco_ros2_control
{
constexpr char PARAM_KP[]{"_kp"};
constexpr char PARAM_KI[]{"_ki"};
constexpr char PARAM_KD[]{"_kd"};
constexpr char PARAM_I_MAX[]{"_i_max"};
constexpr char PARAM_I_MIN[]{"_i_min"};

class MujocoSystem : public MujocoSystemInterface
{
public:
  MujocoSystem();
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

  bool init_sim(
    mjModel *mujoco_model, mjData *mujoco_data, const urdf::Model &urdf_model,
    const hardware_interface::HardwareInfo &hardware_info) override;

  void post_step() override;

  struct JointState
  {
    std::string name;
    double position;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;
    double min_position_command;
    double max_position_command;
    double min_velocity_command;
    double max_velocity_command;
    double min_effort_command;
    double max_effort_command;
    control_toolbox::Pid position_pid;
    control_toolbox::Pid velocity_pid;
    bool is_position_control_enabled{false};
    bool is_velocity_control_enabled{false};
    bool is_effort_control_enabled{false};
    bool is_pid_enabled{false};
    // position_pid_physics_dt: integrate and differentiate the position PID over the physics step it
    // runs at, instead of the time since the last controller update. Gains then mean what they say.
    bool pid_uses_physics_dt{false};
    // effort_pd: per-physics-step joint servo tau = pd_kp*(q_des-q) + pd_kd*(qd_des-qd) + effort_command.
    // pd_kp / pd_kd start at the effort_pd_kp / effort_pd_kd joint parameters and are also the "kp" /
    // "kd" command interfaces when the joint declares them, so a controller can set them per joint
    // and per tick (as Unitree's LowCmd carries kp/kd with every motor command).
    bool is_effort_pd_enabled{false};
    double pd_kp{0.0};
    double pd_kd{0.0};
    joint_limits::JointLimits joint_limits;
    bool is_mimic{false};
    int mimicked_joint_index;
    double mimic_multiplier;
    int mj_joint_type;
    int mj_pos_adr;
    int mj_vel_adr;
    int mj_actuator_id{-1};
  };

  template <typename T>
  struct SensorData
  {
    std::string name;
    T data;
    int mj_sensor_index;
  };

  struct FTSensorData
  {
    std::string name;
    SensorData<Eigen::Vector3d> force;
    SensorData<Eigen::Vector3d> torque;
  };

  struct IMUSensorData
  {
    std::string name;
    SensorData<Eigen::Quaternion<double>> orientation;
    SensorData<Eigen::Vector3d> angular_velocity;
    SensorData<Eigen::Vector3d> linear_acceleration;
  };

  struct BodyStateData
  {
    std::string name;
    std::string body_name;
    int mj_body_id{-1};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  };

  /// Net contact wrench on one body (optionally with its whole subtree) from every active contact
  /// with something outside that set, about the body origin. Simulation-only ground truth, e.g. to
  /// validate a contact estimator; a real robot needs a force sensor for the same signal.
  struct ContactWrenchData
  {
    std::string name;
    std::string body_name;
    int mj_body_id{-1};
    std::vector<char> member;  //!< per MuJoCo body: belongs to the measured set
    bool body_frame{false};    //!< express force/torque in the body frame instead of world
    double contact_threshold{1.0};
    Eigen::Vector3d force{Eigen::Vector3d::Zero()};
    Eigen::Vector3d torque{Eigen::Vector3d::Zero()};
    double normal_force{0.0};  //!< sum of the contact-normal force components [N]
    double contact{0.0};       //!< 1 when normal_force exceeds contact_threshold
  };

private:
  void register_joints(
    const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info);
  void register_sensors(
    const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info);
  bool register_lidar(const hardware_interface::ComponentInfo &sensor);
  void set_initial_pose();
  //! Spawn height of the free joint, if the hardware sets initial_base_height.
  std::optional<double> initial_base_height_;
  void update_body_state_data();
  void update_contact_wrench_data();
  bool register_contact_wrench(const hardware_interface::ComponentInfo &sensor);

  //! Simulation-only episode control for RL training, from a <gpio> with the parameter
  //! mujoco_episode_control. Command interfaces: snapshot and reset are counters - a new value
  //! saves, or restores, the full physics state. Applied at a reset, all relative to the snapshot
  //! and the model as loaded (never compounding over episodes):
  //!   position.{x,y,z} [m, world]     base offset;
  //!   yaw [rad]                        rotation of the robot about the vertical through the base;
  //!   linear_velocity.{x,y,z} [m/s, world], angular_velocity.{x,y,z} [rad/s, base frame];
  //!   friction_scale                   x every geom's sliding friction;
  //!   seed                             seeds the draws below, so a reset is reproducible;
  //!   joint_position_noise [rad], joint_velocity_noise [rad/s]   uniform +- per 1-dof joint;
  //!   link_mass_scale r                every body's mass and inertia x U(1 - r, 1 + r);
  //!   payload_mass [kg], payload_com.{x,y,z} [m]   added to / shifting the centre of mass of the
  //!                                    gpio parameter payload_body (default: the free joint's body).
  //! push is a counter too: a new value applies push_force.{x,y,z} [N, world] at the payload body's
  //! centre of mass for push_duration [s], at any time. State interfaces snapshot / reset / push
  //! echo the last value acted on. Applied in
  //! post_step(), between two physics steps, so the next one starts from it; the simulation time
  //! runs on, so controller clocks never go back.
  struct EpisodeControl
  {
    std::string name;
    double snapshot_command{0.0}, reset_command{0.0}, push_command{0.0};
    double snapshot_state{0.0}, reset_state{0.0}, push_state{0.0};
    double position[3]{0.0, 0.0, 0.0};
    double yaw{0.0};
    double linear_velocity[3]{0.0, 0.0, 0.0};
    double angular_velocity[3]{0.0, 0.0, 0.0};
    double friction_scale{1.0};
    double seed{0.0};
    double joint_position_noise{0.0}, joint_velocity_noise{0.0};
    double link_mass_scale{0.0};
    double payload_mass{0.0};
    double payload_com[3]{0.0, 0.0, 0.0};
    double push_force[3]{0.0, 0.0, 0.0};
    double push_duration{0.1};
    double push_end_time{-1.0};  //!< simulation time the active push ends; < 0: none
    bool has_snapshot{false};
    std::vector<mjtNum> qpos, qvel, act, qacc_warmstart;
    std::vector<mjtNum> sliding_friction;          //!< the model's, per geom
    std::vector<mjtNum> body_mass, body_inertia, body_ipos;  //!< the model's, per body
    std::vector<int> joint_qpos, joint_dof, joint_id;  //!< 1-dof (hinge / slide) joints
    int free_qpos{-1}, free_dof{-1}, payload_body{-1};
  };
  void register_episode_control(const hardware_interface::HardwareInfo &hardware_info);
  void apply_episode_control();
  void reset_episode();
  std::unique_ptr<EpisodeControl> episode_;  // the interfaces point into it
  void get_joint_limits(
    urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits &joint_limits);
  control_toolbox::Pid get_pid_gains(
    const hardware_interface::ComponentInfo &joint_info, std::string command_interface);
  double clamp(double v, double lo, double hi) { return (v < lo) ? lo : (hi < v) ? hi : v; }

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::vector<JointState> joint_states_;
  std::vector<FTSensorData> ft_sensor_data_;
  std::vector<IMUSensorData> imu_sensor_data_;
  std::vector<BodyStateData> body_state_data_;
  std::vector<ContactWrenchData> contact_wrench_data_;
  // unique_ptr: the state interfaces point into each lidar's State, so it must not move.
  std::vector<std::unique_ptr<MujocoLidar>> lidars_;

  mjModel *mj_model_;
  mjData *mj_data_;
  bool use_actuator_effort_command_{false};

  rclcpp::Logger logger_;  // TODO(sangteak601): delete?
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
