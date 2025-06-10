import os
import sys
import launch
import launch_ros.actions
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription([
        # Set RMW implementation to CycloneDDS
        SetEnvironmentVariable(
             name='RMW_IMPLEMENTATION',
             value='rmw_cyclonedds_cpp'
        ),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file:///home/ardupilot/cyclonedds.xml'
        ),
        launch.actions.DeclareLaunchArgument(name='fcu_url'),
        launch.actions.DeclareLaunchArgument(name='gcs_url'),
        launch.actions.DeclareLaunchArgument(name='tgt_system'),
        launch.actions.DeclareLaunchArgument(name='tgt_component'),
        launch.actions.DeclareLaunchArgument(name='log_output', default_value='screen'),
        launch.actions.DeclareLaunchArgument(name='fcu_protocol', default_value='v2.0'),
        launch.actions.DeclareLaunchArgument(name='respawn_mavros', default_value='false')
    ])

    node_params = [
        {'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')},
        {'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')},
        {'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')},
        {'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')},
        {'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')},
        launch.substitutions.LaunchConfiguration('pluginlists_yaml'),
        launch.substitutions.LaunchConfiguration('plugin_global_position_yaml'),
        launch.substitutions.LaunchConfiguration('plugin_setpoint_position_yaml'),
        launch.substitutions.LaunchConfiguration('plugin_sys_time_yaml'),
        launch.substitutions.LaunchConfiguration('plugin_sys_status_yaml'),
        launch.substitutions.LaunchConfiguration('plugin_imu_yaml'),
        launch.substitutions.LaunchConfiguration('plugin_cmd_yaml'),
        #launch.substitutions.LaunchConfiguration('plugin_battery_yaml'),
        #launch.substitutions.LaunchConfiguration('plugin_gps_yaml'),
        #launch.substitutions.LaunchConfiguration('plugin_waypoint_yaml'),
        #launch.substitutions.LaunchConfiguration('plugin_diagnostics_yaml'),
        launch.substitutions.LaunchConfiguration('config_yaml'),
        launch.substitutions.LaunchConfiguration('global_position_config_yaml'),
        launch.substitutions.LaunchConfiguration('setpoint_position_config_yaml'),
        launch.substitutions.LaunchConfiguration('sys_time_config_yaml'),
        launch.substitutions.LaunchConfiguration('sys_status_config_yaml'),
        launch.substitutions.LaunchConfiguration('imu_config_yaml'),
        launch.substitutions.LaunchConfiguration('cmd_config_yaml'),
        #launch.substitutions.LaunchConfiguration('battery_config_yaml'),
        #launch.substitutions.LaunchConfiguration('gps_config_yaml'),
        #launch.substitutions.LaunchConfiguration('waypoint_config_yaml'),
        #launch.substitutions.LaunchConfiguration('diagnostics_config_yaml')
    ]

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=node_params,
        arguments=['--ros-args', '--enclave', '/afs/mavros']
    )

    ld.add_action(mavros_node)
    return ld

if __name__ == '__main__':
    generate_launch_description()