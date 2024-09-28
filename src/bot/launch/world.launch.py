from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pkg_gazebo_ros = os.path.join(os.environ['AMENT_PREFIX_PATH'].split(':')[0], 'share', 'gazebo_ros')
    world_file_path = '/home/ellakiya/ros2_ws/src/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/office.world'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items(),
    )

    return LaunchDescription([
        gazebo
    ])
