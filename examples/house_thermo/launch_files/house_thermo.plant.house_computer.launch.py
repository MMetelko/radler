#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('house_thermo'), 'config', 'house_computer_params.yaml')

    thermostat_node = Node(
        package='house_thermo',
        executable='thermostat',
        namespace='house_thermo',
        name='thermostat',
        output='screen',
        parameters=[config_path]
    )

    thermometer_node = Node(
        package='house_thermo',
        executable='thermometer',
        namespace='house_thermo',
        name='thermometer',
        output='screen',
        parameters=[config_path]
    )

    house_node = Node(
        package='house_thermo',
        executable='house',
        namespace='house_thermo',
        name='house',
        output='screen',
        parameters=[config_path]
    )

    buttons_node = Node(
        package='house_thermo',
        executable='buttons',
        namespace='house_thermo',
        name='buttons',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([
        thermostat_node,
        thermometer_node,
        house_node,
        buttons_node
    ])