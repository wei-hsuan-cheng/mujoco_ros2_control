# mujoco_ros2_control

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for Mujoco simulation, offering the `MujocoSystem` plugin to integrate `ros2_control` with Mujoco. Additionally, it includes a node responsible for initializing the plugin, Mujoco rendering, and the simulation.

## Installation Guide

Follow these steps to install and run the project locally.

### Prerequisites

Make sure you have the following software installed if you are running on the local machine:

- [ROS 2](https://docs.ros.org/en/humble/Installation.html)
- [Mujoco](https://mujoco.org/)
  - Recommended to install the pre-built libraries [here](https://github.com/google-deepmind/mujoco/releases) (following the [doc](https://mujoco.readthedocs.io/en/latest/programming/#)).

### Package Install

Before build this package configure environment variable for mujoco directory.

```bash
export MUJOCO_DIR=<your_path>/mujoco-3.x.x # e.g. mujoco-3.3.7 (depends on your own version)
```

You can now compile the package using the following commands.

```bash
cd mujoco_ros2_control
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select mujoco_ros2_control mujoco_ros2_control_demos
```

Run demos:

```bash
# diff_drive demo launch
ros2 launch mujoco_ros2_control_demos diff_drive.launch.py
# Open another terminal and run
ros2 run mujoco_ros2_control_demos example_diff_drive
```

### Troubleshooting
- If could not find `glfw3` package configuration, install manually.
  1. Download the [GLFW source package](https://www.glfw.org/download.html).
  2. Make and build it (*e.g.* here we use `glfw-3.4`; it can be other versions as well)
      ```bash
      cmake -S <your_path>/glfw-3.4 -B <your_path>/glfw-3.4/build
      cd <your_path>/glfw-3.4/build
      make
      sudo make install
      ```
  
- **DO NOT** use `mujoco_vendor`. It will conflit with the way we configure the environment.

## Usage

See the [documentation](doc/index.rst) for usage.

## Docker

A basic containerized workflow is provided to test this package in isolation.
For more information refer to the [docker documentation](docker/RUNNING_IN_DOCKER.md).

## Future Work

Here are several potential areas for future improvement:

1. **Sensors:** Implement IMU sensors, and range sensors.

2. **Loading Model From URDF:** Implement direct loading of models from URDF, eliminating the need to convert URDF files to XML.

Feel free to suggest ideas for new features or improvements.
