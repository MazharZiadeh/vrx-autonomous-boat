#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '',
                'ssl': False,
                'certfile': '',
                'keyfile': '',
                'authenticate': False,
                'fragment_timeout': 600,  # INTEGER
                'delay_between_messages': 0.0,  # DOUBLE (float)
                'unregister_timeout': 10.0,  # DOUBLE (float) - NOT integer!
            }],
            output='screen'
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'
        ),
    ])
