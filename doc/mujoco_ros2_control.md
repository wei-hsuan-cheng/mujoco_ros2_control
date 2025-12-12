# MuJoCo + ros2_control Integration (Architecture & Real‑Time Aspects)

This note summarizes how the core library `mujoco_ros2_control` integrates MuJoCo with `ros2_control`, and how the example stack in this repo builds on that to stay efficient and reasonably real‑time. Focusing on:

- How MuJoCo’s physics loop is coupled to `ros2_control`’s read–update–write cycle.
- How the hardware abstraction is structured (URDF + `SystemInterface` plugin).
- Where efficiency / decoupling comes from (threading, shared state, minimal copies).

---

## High‑Level Architecture

Key components:

- **MuJoCo engine**
  - `mjModel`, `mjData` allocated and owned by the main node  
    [`mujoco_ros2_control_node.cpp`](../mujoco_ros2_control/src/mujoco_ros2_control_node.cpp).
  - Simulation is advanced by repeated calls to `mj_step1` / `mj_step2`.
- **ros2_control integration**
  - `MujocoSystemInterface` defines an abstract MuJoCo‑aware `SystemInterface`:  
    [`mujoco_system_interface.hpp`](../mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system_interface.hpp).
  - `MujocoSystem` implements it and exposes standard state/command interfaces:  
    [`mujoco_system.hpp`](../mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp),  
    [`mujoco_system.cpp`](../mujoco_ros2_control/src/mujoco_system.cpp).
  - `MujocoRos2Control` owns the `controller_manager` and schedules read–update–write calls synchronised with MuJoCo time:  
    [`mujoco_ros2_control.hpp`](../mujoco_ros2_control/include/mujoco_ros2_control.hpp),  
    [`mujoco_ros2_control.cpp`](../mujoco_ros2_control/src/mujoco_ros2_control.cpp).
- **Rendering & cameras**
  - `MujocoRendering` and `MujocoCameras` share the same `mjModel`/`mjData`, but are purely visualization/sensor publishers:  
    [`mujoco_rendering.cpp`](../mujoco_ros2_control/src/mujoco_rendering.cpp),  
    [`mujoco_cameras.cpp`](../mujoco_ros2_control/src/mujoco_cameras.cpp).

In the main node:

```cpp
// mujoco_ros2_control_node.cpp: main loop
auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);
mujoco_control.init();
...
while (rclcpp::ok() && !rendering->is_close_flag_raised()) {
  mjtNum simstart = mujoco_data->time;
  while (mujoco_data->time - simstart < 1.0 / 60.0) {
    mujoco_control.update();          // advance physics + run controllers
  }
  rendering->update();                // draw MuJoCo scene
  cameras->update(...);               // publish camera images at ~6 Hz
}
```

The control side is thus fully encapsulated inside `MujocoRos2Control`, while the node itself only drives the outer simulation/rendering loop.

---

## Hardware Abstraction: MuJoCo as a ros2_control System

### MuJoCo‑Aware System Interface

The core abstraction is:

- [`MujocoSystemInterface`](../mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system_interface.hpp)

```cpp
class MujocoSystemInterface : public hardware_interface::SystemInterface {
public:
  virtual bool init_sim(
    mjModel* mujoco_model, mjData* mujoco_data,
    const urdf::Model& urdf_model,
    const hardware_interface::HardwareInfo& hardware_info) = 0;
};
```

This extends the standard `SystemInterface` with an `init_sim(...)` hook that:

- Provides direct access to **MuJoCo model/data** (`mjModel*`, `mjData*`).
- Passes the parsed **URDF model** and `HardwareInfo` (joints, sensors) to map ROS control interfaces onto MuJoCo DOFs/sensors.

Any hardware plugin that wants to simulate on top of MuJoCo just implements this interface and is loaded via pluginlib.

### Concrete Implementation: `MujocoSystem`

The default plugin used in this repo is:

- [`MujocoSystem`](../mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp)
- Exported in [`mujoco_system.cpp`](../mujoco_ros2_control/src/mujoco_system.cpp)

It implements:

- `export_state_interfaces()` / `export_command_interfaces()`:
  - Build `StateInterface` / `CommandInterface` objects backed by **pointers into a joint‑state struct**, not copies.
  - Those structs (`JointState`, `FTSensorData`, `IMUSensorData`) keep:
    - Joint position/velocity/effort and their commands.
    - PID controllers (if enabled).
    - Mappings to MuJoCo indices: `mj_pos_adr`, `mj_vel_adr`, and sensor addresses.
- `read()`:
  - Copies MuJoCo state into these joint/sensor structs:
    - `qpos[qpos_adr]` → `JointState::position`.
    - `qvel[qvel_adr]` → `JointState::velocity`.
    - `qfrc_applied[qvel_adr]` → `JointState::effort`.
    - IMU/FT sensors: slices of `mjData::sensordata` into `Eigen` vectors/quaternions.
- `write()`:
  - Applies commanded torques/positions/velocities back to MuJoCo:
    - Effort mode: clamps `effort_command` using URDF + ros2_control limits, writes into `qfrc_applied`.
    - Position/velocity mode:
      - With PID: PD/PID in joint space to compute torques → `qfrc_applied`.
      - Without PID: direct write into `qpos` / `qvel` (kinematic mode).

The mapping between URDF/ros2_control and MuJoCo is set up once in `init_sim()`:

- `register_joints()`:
  - For each `hardware_info.joints[i]`, find the MuJoCo joint with `mj_name2id(mjOBJ_JOINT, joint.name)`.
  - Cache:
    - `mj_model_->jnt_qposadr[id]` and `jnt_dofadr[id]` into `mj_pos_adr`, `mj_vel_adr`.
    - URDF joint limits into a `joint_limits` struct.
  - Build `StateInterface` / `CommandInterface` pointing into `JointState` fields.
  - Parse ROS parameters for PID gains (`position_kp`, `velocity_kp`, etc.).
- `register_sensors()`:
  - Uses naming conventions:
    - FT sensors end with `_fts` (ROS sensor name) and map to `*_force` / `*_torque` MuJoCo sensors.
    - IMUs end with `_imu` and map to quaternion + gyro + accelerometer MuJoCo sensors.
  - For each, it:
    - Resolves sensor IDs via `mj_name2id(mjOBJ_SENSOR, ...)`.
    - Stores `sensor_adr[id]` as the index into `mjData::sensordata`.
    - Creates `StateInterface`s for the relevant components (e.g., `force.x`, `orientation.w`).

Because all ROS interfaces point directly into these small structs, and those structs are updated in place from MuJoCo, there is no heavy copying in or out of ROS messages at the control‑loop level.

---

## Coupling MuJoCo and ros2_control Loops

`MujocoRos2Control` is the glue between MuJoCo time and the controller manager:

- File: [`mujoco_ros2_control.cpp`](../mujoco_ros2_control/src/mujoco_ros2_control.cpp)

### Initialisation

1. **Get URDF / ros2_control description**
   - `get_robot_description()`:
     - Tries to read the `robot_description` parameter from a temporary node.
     - If missing, subscribes to a `robot_description` topic and blocks until the message arrives.
2. **Parse ros2_control hardware from URDF**
   - Uses `hardware_interface::parse_control_resources_from_urdf(urdf_string)` to get a list of `HardwareInfo`.
   - Allocates a `hardware_interface::ResourceManager` and calls `load_urdf` on it.
3. **Load MuJoCo hardware plugin(s)**
   - Creates a `pluginlib::ClassLoader<MujocoSystemInterface>`.
   - For each `HardwareInfo`:
     - Instantiates the class named in `hardware.hardware_class_type` (e.g. `mujoco_ros2_control/MujocoSystem`).
     - Calls `init_sim(mj_model_, mj_data_, urdf_model, hardware)` to wire it to MuJoCo.
     - Imports the component into the `ResourceManager` and marks it as ACTIVE.
4. **Create controller manager**
   - Creates a `controller_manager::ControllerManager` with the configured resource manager.
   - Reads the `update_rate` parameter and sets `control_period_ = 1/update_rate`.
   - Forces `use_sim_time = true`.
   - Spins the controller manager in a **separate thread** using a `MultiThreadedExecutor`, so controller callbacks remain responsive.

### Per‑Step Update

`MujocoRos2Control::update()` is called inside the MuJoCo loop:

```cpp
void MujocoRos2Control::update() {
  // 1) Build ROS time from MuJoCo's mjData::time
  double sim_time = mj_data_->time;
  rclcpp::Time sim_time_ros(sec, nsec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);   // publish /clock (sim time)

  // 2) Advance MuJoCo one "half" step: integrate dynamics
  mj_step1(mj_model_, mj_data_);

  // 3) Run ros2_control at its own frequency
  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // 4) Always write the latest control outputs into MuJoCo
  controller_manager_->write(sim_time_ros, sim_period);

  // 5) Finish MuJoCo step (constraint resolution, etc.)
  mj_step2(mj_model_, mj_data_);
}
```

This yields:

- **Single source of truth for time**: `mjData::time`, converted to `/clock`. All ROS nodes configured with `use_sim_time` will use that.
- **Decoupled frequencies**:
  - MuJoCo can run with small internal timesteps (driven by the outer loop).
  - `ros2_control` only runs when `sim_period >= control_period_` (e.g. 100 Hz).
- **Tight coupling where needed**:
  - `read()` always sees **fresh** MuJoCo state.
  - `write()` is always called after `update()`, before `mj_step2`, so controllers’ outputs are applied to the next physics step.

Importantly, there is no ROS messaging in this hot path—only in‑process method calls and direct memory access—so the control loop stays light.

---

## Efficiency & Decoupling Design

Compared to the custom `robot_runtime` + MuJoCo backend in `wb_humanoid_mpc`, here the efficiency/decoupling is organised explicitly around `ros2_control`:

### 1. Clear Separation of Concerns

- **Physics backend**: MuJoCo, driven entirely by `mj_step1/2` on `mjModel`/`mjData`.
- **Hardware abstraction**: `MujocoSystemInterface` / `MujocoSystem`:
  - Knows how to read/write MuJoCo state, but not about controllers.
- **Controllers**: standard `ros2_control` controllers loaded by `controller_manager`.

This means:

- You can swap controllers (e.g. joint trajectory, impedance, custom controllers) without touching the MuJoCo code.
- You can in principle swap `MujocoSystem` with another implementation that uses a different mapping or different sensor set, still reusing the same controllers.

### 2. Minimal Copying / Direct Memory Access

- Joint and sensor data are stored in small structs owned by `MujocoSystem`.
- `StateInterface` / `CommandInterface` objects hold **raw pointers** to those fields.
- `read()` and `write()` directly operate on `mjData::qpos`, `qvel`, `qfrc_applied`, and `sensordata`.
- No intermediate ROS messages are involved in the control loop; messages are only used by:
  - Controllers themselves (topics/services).
  - Camera/image pipelines and any logging/visualization outside the control loop.

This is analogous to the thread‑safe shared state pattern in `wb_humanoid_mpc`, but realised through `ros2_control`’s pointer‑based interfaces and MuJoCo’s own arrays.

### 3. Threading Strategy

- The **main thread**:
  - Runs the MuJoCo simulation loop and `MujocoRos2Control::update()`.
  - Runs rendering and camera updates at lower rates.
- The **controller manager**:
  - Spins in a separate thread via a `MultiThreadedExecutor`.
  - Reacts to parameter changes, controller lifecycle operations, etc., without blocking the physics loop.

The interface between the two is a handful of `controller_manager_->read/update/write` calls and shared `ResourceManager` state, which are designed to be cheap in `ros2_control`.

### 4. Flexible Control Modes with PID

For each joint, `MujocoSystem` can enable:

- Position / velocity / effort control, and optionally PID:
  - Position / velocity PID uses joint position/velocity errors to compute torques.
  - Effort mode directly applies torques, with limits enforced by URDF + `min/max` in the command interfaces.

PID gains are read from joint parameters (e.g. `position_kp`, `velocity_kp`, etc.), so you can tune them in your URDF/xacro or controller configs without modifying the integration code.

---

## How the Examples Use This

In this repo:

- `panda_resources/panda_mujoco` provides MuJoCo + URDF description of the Panda robot.
- `panda_resources/panda_moveit_config` and `interactive_marker` add MoveIt / RViz integration.
- `peg_in_hole` implements a concrete control demo that talks to the Panda via:
  - `ros2_control` controllers powered by `MujocoSystem`.
  - Standard ROS topics/services.

Because the MuJoCo integration is implemented as a `SystemInterface` plugin, the examples do not need to care about MuJoCo internals:

- They only see a `ros2_control`‑compatible robot with joints and sensors.
- Real‑time physics and timing are handled by `mujoco_ros2_control_node` + `MujocoRos2Control`.

This mirrors the design from `wb_humanoid_mpc`:

- There, **robot_runtime** decouples physics backend (MuJoCo vs “dummy” internal integrator) from the MPC and controllers.
- Here, `ros2_control` plays the same role, with MuJoCo wrapped behind a `SystemInterface` and the controller manager.

If you want to build your own real‑time‑ish experiments on top:

- Keep heavy work (logging, camera processing, planning) off the hot `update()` path.
- Stick to `ros2_control` controllers, so they benefit from this shared architecture automatically.

