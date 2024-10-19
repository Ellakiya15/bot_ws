import launch
import launch_ros.actions
import os

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='bot').find('bot')
    urdfModelPath = os.path.join(pkgPath, 'urdf', 'trial.urdf')
    
    # Ensure the URDF file exists
    if not os.path.exists(urdfModelPath):
        raise FileNotFoundError(f"URDF file not found: {urdfModelPath}")

    # Load the URDF file
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return launch.LaunchDescription([
        robot_state_publisher_node
    ])
