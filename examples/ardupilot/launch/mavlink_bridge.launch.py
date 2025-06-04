# mavlink_bridge.launch.py
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Define the bridge URL parameter
        DeclareLaunchArgument(
            'mavlink_bridge_url',
            default_value='udp://@:14560?to=127.0.0.1:14551'
        ),
        
        # Launch the GCS bridge node
        Node(
            package='mavros',
            executable='mavros_gcs_bridge',  # ROS2 uses 'executable' instead of 'type'
            name='mavlink_bridge',
            parameters=[{
                'gcs_url': LaunchConfiguration('mavlink_bridge_url')
            }]
        )
        # Add a specific bridge for fence visualization
        Node(
            package='mavros',
            executable='mavros_gcs_bridge',
            name='mavproxy_fence_bridge',
            parameters=[{
                'gcs_url': 'udp://@:14570?to=127.0.0.1:14550'
            }]
        )
    ])
