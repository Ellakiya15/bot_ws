from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directories of packages and config files
    slam_toolbox_launch_dir = get_package_share_directory('slam_toolbox')
    bot_package_dir = get_package_share_directory('bot')
    
    # Declare the parameter for SLAM config file
    declare_mapper_online_async_param_cmd = DeclareLaunchArgument(
        'async_param',
        default_value=os.path.join(bot_package_dir, 'config', 'slam_toolbox_config.yaml'),
        description='Path to the SLAM Toolbox configuration file'
    )

    # Include the SLAM Toolbox online_async launch file
    mapper_online_async_param_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'params_file': LaunchConfiguration('async_param')}.items(),
    )

    return LaunchDescription([
        # Declare the SLAM parameter argument
        declare_mapper_online_async_param_cmd,

        # Launch the simulation world (Gazebo or any other)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [bot_package_dir, '/launch/world.launch.py']  # Replace with your Gazebo world launch file
            )
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Launch robot_state_publisher to publish URDF and TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Launch the SLAM Toolbox with the specified config file
        mapper_online_async_param_launch
    ])



