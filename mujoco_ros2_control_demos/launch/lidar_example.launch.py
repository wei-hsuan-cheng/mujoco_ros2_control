"""Standalone 3D lidar demo: a Livox Mid-360 on a mast in a walled room.

    ros2 launch mujoco_ros2_control_demos lidar_example.launch.py
    ros2 launch mujoco_ros2_control_demos lidar_example.launch.py rviz:=false headless:=true

Then, in another terminal:

    ros2 topic hz /livox/lidar                        # ~10 Hz
    ros2 control list_hardware_interfaces             # mid360_lidar/... state interfaces
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    demos_path = get_package_share_directory('mujoco_ros2_control_demos')

    doc = xacro.parse(open(os.path.join(demos_path, 'urdf', 'test_lidar.xacro.urdf')))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    use_sim_time = {'use_sim_time': True}

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            os.path.join(demos_path, 'config', 'lidar_example.yaml'),
            use_sim_time,
            {'mujoco_model_path': os.path.join(demos_path, 'mujoco_models', 'test_lidar.xml')},
            {'mujoco_headless': ParameterValue(LaunchConfiguration('headless'), value_type=bool)},
        ],
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[use_sim_time, robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[use_sim_time],
        arguments=['-d', os.path.join(demos_path, 'launch', 'lidar_demo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz on the cloud'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run MuJoCo without its viewer window'),
        node_mujoco_ros2_control,
        node_robot_state_publisher,
        rviz_node,
    ])
