#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Serial device path for the Raspberry Pi Pico'
        ),
        
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose logging'
        ),
        
        # Log launch information
        LogInfo(
            msg=['Starting micro-ROS agent for BNO085 IMU driver']
        ),
        
        LogInfo(
            msg=[
                'Device: ', LaunchConfiguration('device'),
                ', Baud: ', LaunchConfiguration('baud')
            ]
        ),
        
        # micro-ROS agent node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=[
                'serial',
                '--dev', LaunchConfiguration('device'),
                '--baud', LaunchConfiguration('baud'),
            ],
            output='screen',
            parameters=[
                {'verbose': LaunchConfiguration('verbose')}
            ],
            respawn=True,
            respawn_delay=2.0
        )
    ])
