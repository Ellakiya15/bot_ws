#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_bot = get_package_share_directory('bot')
    install_dir = get_package_prefix('bot')
    gazebo_models_path = os.path.join(pkg_bot, 'models')
    world_directory = os.path.join(pkg_bot,'world','no_roof_small_warehouse.world')
    urdf_file = os.path.join(pkg_bot, 'urdf', 'trial.urdf')

    # Set GAZEBO_MODEL_PATH and GAZEBO_PLUGIN_PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    return LaunchDescription([
        # Launch the Gazebo simulator with the specified world
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_bot, 'world', 'no_roof_small_warehouse.world')],
            description='SDF world file'),

        ExecuteProcess(
            cmd=['gazebo', world_directory, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'your_robot_name'],
            output='screen'
        )
    ])
