#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Serial device path for the Raspberry Pi Pico'
        ),
        
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz for IMU visualization'
        ),
        
        DeclareLaunchArgument(
            'start_rqt_plot',
            default_value='false',
            description='Start rqt_plot for real-time data plotting'
        ),
        
        DeclareLaunchArgument(
            'use_alternative_viz',
            default_value='false',
            description='Use alternative visualization instead of RViz (useful for snap RViz issues)'
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
        
        # RViz for visualization
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(
                get_package_share_directory('bno085_driver'), 
                'rviz', 
                'bno085_imu.rviz'
            )],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            output='screen'
        ),
        
        # rqt_plot for real-time data visualization
        ExecuteProcess(
            cmd=[
                'rqt_plot',
                '/imu/data/angular_velocity/x',
                '/imu/data/angular_velocity/y', 
                '/imu/data/angular_velocity/z',
                '--title', 'IMU Angular Velocity'
            ],
            condition=IfCondition(LaunchConfiguration('start_rqt_plot')),
            output='screen'
        ),
        
        # Alternative visualization - topic echo (when RViz has issues)
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '/imu/data'
            ],
            condition=IfCondition(LaunchConfiguration('use_alternative_viz')),
            output='screen'
        )
    ])
