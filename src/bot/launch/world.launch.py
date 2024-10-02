from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the world file
    world_file = os.path.join(
        get_package_share_directory('bot'),
        'world',
        'bookstore.world'
    )
    
    # Get the path to the models directory
    models_dir = os.path.join(
        get_package_share_directory('bot'),
        'models'
    )

    # Set the GAZEBO_MODEL_PATH to include the models directory
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir
    )

    # Launch Gazebo with the world file
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,  # Set the models path
        gazebo,             # Launch Gazebo
    ])
