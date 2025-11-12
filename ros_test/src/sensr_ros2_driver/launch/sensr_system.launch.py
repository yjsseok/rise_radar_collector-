#!/usr/bin/env python3
"""
SENSR Complete System Launch File
드라이버 + Bag 레코더를 함께 실행
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='112.133.37.122',
        description='SENSR server IP address'
    )
    
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='./sensr_bags',
        description='Directory to save bag files'
    )
    
    bag_duration_arg = DeclareLaunchArgument(
        'bag_duration',
        default_value='60',
        description='Duration of each bag file in seconds'
    )
    
    enable_recorder_arg = DeclareLaunchArgument(
        'enable_recorder',
        default_value='true',
        description='Enable bag recording'
    )
    
    # SENSR driver node
    sensr_driver_node = Node(
        package='sensr_ros2_driver',
        executable='sensr_driver_node.py',
        name='sensr_driver',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'output_port': 5050,
            'pointcloud_port': 5051,
            'frame_id': 'sensr',
            'publish_rate': 10.0,
        }]
    )
    
    # SENSR bag recorder node (5초 후 시작)
    sensr_bag_recorder_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='sensr_ros2_driver',
                executable='sensr_bag_recorder.py',
                name='sensr_bag_recorder',
                output='screen',
                parameters=[{
                    'output_directory': LaunchConfiguration('output_directory'),
                    'bag_duration': LaunchConfiguration('bag_duration'),
                    'max_bag_size': 1024,  # MB
                    'topics': [
                        '/sensr/pointcloud',
                        '/sensr/objects',
                        '/sensr/events',
                        '/sensr/diagnostics'
                    ]
                }],
                condition=LaunchConfiguration('enable_recorder')
            )
        ]
    )
    
    return LaunchDescription([
        host_arg,
        output_directory_arg,
        bag_duration_arg,
        enable_recorder_arg,
        sensr_driver_node,
        sensr_bag_recorder_node,
    ])