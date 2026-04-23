#!/usr/bin/env python3
"""
Launch file for Barista Robot System with GUI
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('assem6')
    urdf_file = os.path.join(pkg_path, 'urdf', 'Assem6.urdf')
    rviz_config = os.path.join(pkg_path, 'config', 'rviz.rviz')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        
        # Barista Robot GUI Controller
        Node(
            package='assem6',
            executable='barista_gui.py',
            name='barista_gui',
            output='screen',
        ),
        
        # RViz for visualization with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
