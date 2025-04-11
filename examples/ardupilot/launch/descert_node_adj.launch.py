import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='fcu_url'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gcs_url'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tgt_system'
        ),
        launch.actions.DeclareLaunchArgument(
            name='tgt_component'
        ),
        launch.actions.DeclareLaunchArgument(
            name='pluginlists_yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='config_yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='log_output',
            default_value='screen'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fcu_protocol',
            default_value='v2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='respawn_mavros',
            default_value='false'
        ),
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            #output=launch.substitutions.LaunchConfiguration('log_output'),
            parameters=[
                {
                    'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')
                },
                {
                    'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')
                },
                {
                    'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')
                },
                {
                    'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')
                },
                {
                    'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')
                },
                launch.substitutions.LaunchConfiguration('pluginlists_yaml'),
                launch.substitutions.LaunchConfiguration('config_yaml')
            ]
        )
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            name='global_position',
            #output=launch.substitutions.LaunchConfiguration('log_output'),
            parameters=[
                {
                    'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')
                },
                {
                    'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')
                },
                {
                    'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')
                },
                {
                    'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')
                },
                {
                    'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')
                },
                launch.substitutions.LaunchConfiguration('plugin_global_position_yaml'),
                launch.substitutions.LaunchConfiguration('global_position_config_yaml')
            ],
            arguments=['--ros-args', '--enclave', '/afs/mavros']
        )
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            name='sys_time',
            #output=launch.substitutions.LaunchConfiguration('log_output'),
            parameters=[
                {
                    'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')
                },
                {
                    'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')
                },
                {
                    'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')
                },
                {
                    'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')
                },
                {
                    'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')
                },
                launch.substitutions.LaunchConfiguration('plugin_sys_time_yaml'),
                launch.substitutions.LaunchConfiguration('sys_time_config_yaml')
            ],
            arguments=['--ros-args', '--enclave', '/afs/mavros']
        )
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            name='sys_status',
            #output=launch.substitutions.LaunchConfiguration('log_output'),
            parameters=[
                {
                    'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')
                },
                {
                    'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')
                },
                {
                    'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')
                },
                {
                    'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')
                },
                {
                    'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')
                },
                launch.substitutions.LaunchConfiguration('plugin_sys_status_yaml'),
                launch.substitutions.LaunchConfiguration('sys_status_config_yaml')
            ],
            arguments=['--ros-args', '--enclave', '/afs/mavros']
        )
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            name='imu',
            #output=launch.substitutions.LaunchConfiguration('log_output'),
            parameters=[
                {
                    'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')
                },
                {
                    'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')
                },
                {
                    'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')
                },
                {
                    'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')
                },
                {
                    'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')
                },
                launch.substitutions.LaunchConfiguration('plugin_imu_yaml'),
                launch.substitutions.LaunchConfiguration('imu_config_yaml')
            ],
            arguments=['--ros-args', '--enclave', '/afs/mavros']
        )
        launch_ros.actions.Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            name='cmd',
            #output=launch.substitutions.LaunchConfiguration('log_output'),
            parameters=[
                {
                    'fcu_url': launch.substitutions.LaunchConfiguration('fcu_url')
                },
                {
                    'gcs_url': launch.substitutions.LaunchConfiguration('gcs_url')
                },
                {
                    'target_system_id': launch.substitutions.LaunchConfiguration('tgt_system')
                },
                {
                    'target_component_id': launch.substitutions.LaunchConfiguration('tgt_component')
                },
                {
                    'fcu_protocol': launch.substitutions.LaunchConfiguration('fcu_protocol')
                },
                launch.substitutions.LaunchConfiguration('plugin_cmd_yaml'),
                launch.substitutions.LaunchConfiguration('cmd_config_yaml')
            ],
            arguments=['--ros-args', '--enclave', '/afs/mavros']
        )       
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
