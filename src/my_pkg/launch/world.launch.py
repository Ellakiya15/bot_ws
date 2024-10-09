from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the package share directory
    package_share_dir = get_package_share_directory('my_pkg')
    warehouse_world = os.path.join(package_share_dir, 'worlds', 'no_roof_small_warehouse.world')

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', warehouse_world, '-entity', 'warehouse'],
            output='screen',
        ),
    ])
