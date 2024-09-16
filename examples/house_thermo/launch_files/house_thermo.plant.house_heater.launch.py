#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the parameter file
    parameters_file_path = os.path.join(get_package_share_directory('house_thermo'), 'config', 'house_heater_params.yaml')

    # Define the Node action
    heater_node = Node(
        package='house_thermo',
        executable='heater',
        name='heater',
        namespace='house_thermo',
        output='screen',
        parameters=[parameters_file_path]
    )

    return LaunchDescription([
        heater_node
    ])