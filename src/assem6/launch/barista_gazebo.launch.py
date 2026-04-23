#!/usr/bin/env python3
"""
Simple Gazebo Launch for Barista Robot with GUI
Uses Gazebo's built-in joint state publisher instead of ros2_control
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('assem6')
    
    # Paths - use the original URDF with Gazebo joint state publisher plugin
    urdf_file = os.path.join(pkg_path, 'urdf', 'Assem6.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'barista_world.sdf')
    rviz_config = os.path.join(pkg_path, 'config', 'rviz.rviz')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen',
    )

    # Gazebo Server only (no GUI - more stable)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Gazebo Client (GUI) - run separately with delay
    gzclient = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen'
            )
        ]
    )

    # Spawn robot in Gazebo
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', urdf_file,
                    '-entity', 'assem6',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # Barista GUI - delay to start after Gazebo loads
    barista_gui = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='assem6',
                executable='barista_gui.py',
                name='barista_gui',
                output='screen',
            )
        ]
    )

    # RViz
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        barista_gui,
        rviz,
    ])
