# mujoco_ros2_control

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for Mujoco simulation, offering the `MujocoSystem` plugin to integrate `ros2_control` with Mujoco. Additionally, it includes a node responsible for initializing the plugin, Mujoco rendering, and the simulation.

## Installation Guide

Follow these steps to install and run the project locally.

### Prerequisites

Make sure you have [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) installed. This branch targets ros2_control 4.x; ROS 2 Humble is on the `humble` branch.

### Package Install

```bash
git clone https://github.com/wei-hsuan-cheng/mujoco_ros2_control.git

cd <workspace_dir>
source /opt/ros/${ROS_DISTRO}/setup.bash
NUM_JOBS=2 && \
  export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
  export MAKEFLAGS=-j${NUM_JOBS} && \
  export NINJAFLAGS=-j${NUM_JOBS} && \
  colcon build --symlink-install \
    --packages-up-to mujoco_ros2_control mujoco_ros2_control_demos \
    --executor sequential --parallel-workers ${NUM_JOBS} \
    --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash
```

(Optional) To pin a different prebuilt version pass `-DMUJOCO_VERSION=3.x.x` in `--cmake-args` (**3.11.0 or newer**; `mjv_moveCamera` changed signature in 3.11.0).

Run demos:

```bash
# diff_drive demo launch
ros2 launch mujoco_ros2_control_demos diff_drive.launch.py
# Open another terminal and run
ros2 run mujoco_ros2_control_demos example_diff_drive
```

### Troubleshooting

- **DO NOT** use `mujoco_vendor`. It will conflit with the way we configure the environment. Run this to make sure the mujoco env is correct
  ```bash
  # Terminal command
  ldd install/mujoco_ros2_control/lib/mujoco_ros2_control/mujoco_ros2_control | grep mujoco
  # It should returns this (e.g., for mujoco-3.13.0)
  libmujoco.so.3.13.0 => <your_mujoco_path>/mujoco-3.13.0/lib/libmujoco.so.3.13.0 (0x0000709b0f376000)
  ```

## Usage

See the [documentation](doc/index.rst) for usage.


### Floating-Base Feedback Frames

Body-state hardware interfaces and ground-truth odometry report pose at the regular link origin, with twist expressed in that link's local axes.
Both paths use `mj_objectVelocity(..., mjOBJ_XBODY, ..., 1)` to match the published `xpos/xquat` pose.
MuJoCo's `mjOBJ_BODY` instead uses the center of mass and principal-inertia axes for local velocity output, which can differ substantially from the robot link frame.
Using BODY twist with link pose can therefore corrupt angular feedback even when the rendered model and positions look correct.
See the [MuJoCo source](https://github.com/google-deepmind/mujoco/blob/3.3.2/src/engine/engine_support.c#L1166) and the regression test in `mujoco_ros2_control/tests/test_body_twist_frames.py`.
With Python MuJoCo and NumPy installed, run `python3 -m unittest discover -s mujoco_ros2_control/tests -v` from this repository.


## Docker

A basic containerized workflow is provided to test this package in isolation.
For more information refer to the [docker documentation](docker/RUNNING_IN_DOCKER.md).

## Future Work

Here are several potential areas for future improvement:

1. **Sensors:** Implement IMU sensors, and range sensors.

2. **Loading Model From URDF:** Implement direct loading of models from URDF, eliminating the need to convert URDF files to XML.

Feel free to suggest ideas for new features or improvements.
