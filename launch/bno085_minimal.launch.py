#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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
            'show_data',
            default_value='false',
            description='Show IMU data in terminal'
        ),
        
        # micro-ROS agent node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=[
                'serial',
                '--dev', LaunchConfiguration('device'),
                '--baud', '115200',
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),
        
        # Static transform publisher (IMU to base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_transform',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link'
            ],
            output='screen'
        ),
        
        # Show IMU data in terminal (alternative to RViz)
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '/imu/data', 
                '--field', 'angular_velocity'
            ],
            condition=IfCondition(LaunchConfiguration('show_data')),
            output='screen'
        )
    ])
