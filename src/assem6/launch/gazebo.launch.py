import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('assem6')
    urdf_file = os.path.join(pkg_share, 'urdf', 'Assem6.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Remove XML declaration to avoid encoding issues with spawn_entity
    if robot_description.startswith('<?xml'):
        robot_description = robot_description.split('?>', 1)[1].strip()

    # Set Gazebo model path to find meshes
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.path.join(pkg_share, '..')]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Spawn robot in Gazebo (z=0 since world_fixed joint handles the height)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'assem6',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz
    ])
