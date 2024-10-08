import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Path to your robot description
    robot_description_path = os.path.join(
        get_package_share_directory('bot'),
        'urdf',
        'trial.urdf'
    )

    # Command to load URDF into parameter
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', robot_description_path])}]
    )

    # Spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', robot_description_path],
        output='screen'
    )

    # Launch Gazebo
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    return LaunchDescription([
        gazebo_cmd,
        robot_state_publisher_cmd,
        spawn_entity_cmd,
    ])

