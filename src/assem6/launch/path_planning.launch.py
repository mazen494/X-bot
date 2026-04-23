#!/usr/bin/env python3
"""
Launch file for assem6 robot with path planning capabilities.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('assem6')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem6.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Remove XML declaration to avoid encoding issues
    if robot_description.startswith('<?xml'):
        robot_description = robot_description.split('?>', 1)[1].strip()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Path Planner Node
    path_planner = Node(
        package='assem6',
        executable='path_planner_node.py',
        name='path_planner',
        output='screen'
    )

    # RViz for visualization
    rviz_config = os.path.join(pkg_share, 'config', 'path_planning.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        path_planner,
        rviz
    ])
