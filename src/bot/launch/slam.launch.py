from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_toolbox_launch_dir = get_package_share_directory('slam_toolbox')
    bot_package_dir = get_package_share_directory('bot')

    return LaunchDescription([
        # Launch Gazebo or your simulation environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [bot_package_dir, '/launch/world.launch.py']  # Replace with your Gazebo launch file
            )
        ),
        
        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Launch your robot's URDF description, sensors, etc.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Launch SLAM Toolbox with the config file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [slam_toolbox_launch_dir, '/launch/online_async_launch.py']
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': bot_package_dir + '/config/slam_toolbox_config.yaml'  # Add config file here
            }.items(),
        )
    ]) 


