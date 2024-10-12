import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            arguments=['--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Launch your RL training script
        launch_ros.actions.Node(
            package='bot',
            executable='robot_rl_training.py',
            name='robot_rl_training_node',
            output='screen'
        )
    ])
