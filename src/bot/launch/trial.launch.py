import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='bot').find('bot')
    urdfModelPath = os.path.join(pkgPath, 'urdf/trial.urdf')
    
    # Read the URDF file content
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    # Define the robot description parameter
    params = {'robot_description': robot_desc}
    
    # Node for robot_state_publisher (publishes robot states from URDF)
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Node for joint_state_publisher (publishes joint states)
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))  # Uses GUI only if 'gui' is False
    )
    
    # Node for joint_state_publisher_gui (publishes joint states with GUI)
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))  # Uses GUI if 'gui' is True
    )
    
    # RViz node for visualization
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Launch description with nodes
    return launch.LaunchDescription([
        # Declare the 'gui' argument, default is 'True'
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag for joint_state_publisher_gui'),
        # Declare the URDF model path argument
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdfModelPath,
                                             description='Path to the URDF model file'),
        # Add the nodes to the launch description
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
