# mujoco_ros2_control

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for Mujoco simulation, offering the `MujocoSystem` plugin to integrate `ros2_control` with Mujoco. Additionally, it includes a node responsible for initializing the plugin, Mujoco rendering, and the simulation.

## Installation Guide

Follow these steps to install and run the project locally.

### Prerequisites

Make sure you have [ROS 2](https://docs.ros.org/en/humble/Installation.html) installed.

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

(Optional) To pin a different prebuilt version pass `-DMUJOCO_VERSION=3.x.x` in `--cmake-args`, or keep using your own install by exporting `MUJOCO_DIR`:

```bash
# optional: only when you want to manage the mujoco prebuilt yourself
cd <any_path>
# Check x86_64 or aarch64
wget -O mujoco-3.3.7-linux-x86_64.tar.gz \
  https://github.com/google-deepmind/mujoco/releases/download/3.3.7/mujoco-3.3.7-linux-x86_64.tar.gz && \
tar -xzf mujoco-3.3.7-linux-x86_64.tar.gz
export MUJOCO_DIR=<any_path>/mujoco-3.x.x # e.g. mujoco-3.3.7 (depends on your own version)
```

Run demos:

```bash
# diff_drive demo launch
ros2 launch mujoco_ros2_control_demos diff_drive.launch.py
# Open another terminal and run
ros2 run mujoco_ros2_control_demos example_diff_drive
```

### Troubleshooting

- If could not find `glfw3` package configuration, install manually
  (*e.g.*, here we use `glfw-3.4`; it can be other versions as well):

  ```bash
  # Download and extract the GLFW source package
  cd <your_path>
  wget https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip && unzip glfw-3.4.zip

  # Configure, build, and install (sudo only needed for the install step)
  sudo cmake -S glfw-3.4 -B glfw-3.4/build
  sudo cmake --build glfw-3.4/build -j
  sudo cmake --install glfw-3.4/build
  ```
  
- **DO NOT** use `mujoco_vendor`. It will conflit with the way we configure the environment. Run this to make sure the mujoco env is correct
  ```bash
  # Terminal command
  ldd install/mujoco_ros2_control/lib/mujoco_ros2_control/mujoco_ros2_control | grep mujoco
  # It should returns this (e.g., for mujoco-3.3.7)
  libmujoco.so.3.3.7 => <your_mujoco_path>/mujoco-3.3.7/lib/libmujoco.so.3.3.7 (0x0000709b0f376000)
  ```

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
