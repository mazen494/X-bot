#!/usr/bin/env python3
"""
Launch file for Assem6 Hardware on Raspberry Pi 4

Starts:
  1. robot_state_publisher (with hardware URDF)
  2. servo_bridge node (PCA9685 servo driver)
  3. barista_gui (on Pi touchscreen)

Usage:
  ros2 launch assem6_hardware hardware.launch.py
  ros2 launch assem6_hardware hardware.launch.py gui:=false    # Headless mode
  ros2 launch assem6_hardware hardware.launch.py config:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    hw_pkg = get_package_share_directory('assem6_hardware')
    assem6_pkg = get_package_share_directory('assem6')

    # Default paths
    default_config = os.path.join(hw_pkg, 'config', 'servo_config.yaml')
    urdf_file = os.path.join(assem6_pkg, 'urdf', 'Assem6_hardware.urdf')

    # Fallback to original URDF if hardware URDF doesn't exist
    if not os.path.exists(urdf_file):
        urdf_file = os.path.join(assem6_pkg, 'urdf', 'Assem6.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to servo_config.yaml'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch barista GUI on touchscreen'
    )

    smoothing_arg = DeclareLaunchArgument(
        'smoothing',
        default_value='true',
        description='Enable servo motion smoothing'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Servo Bridge Node (the hardware driver)
    servo_bridge = Node(
        package='assem6_hardware',
        executable='servo_bridge',
        name='servo_bridge',
        parameters=[{
            'config_file': LaunchConfiguration('config'),
            'update_rate': 50.0,
            'max_speed': 2.0,
            'enable_smoothing': True,
            'smoothing_factor': 0.15,
        }],
        output='screen',
    )

    # Barista GUI (runs on Pi touchscreen)
    barista_gui = Node(
        package='assem6',
        executable='barista_gui.py',
        name='barista_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription([
        config_arg,
        gui_arg,
        smoothing_arg,
        robot_state_publisher,
        servo_bridge,
        barista_gui,
    ])
