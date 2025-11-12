#!/usr/bin/env python3
"""
SENSR ROS2 Driver Launch File
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='112.133.37.122',
        description='SENSR server IP address'
    )
    
    output_port_arg = DeclareLaunchArgument(
        'output_port',
        default_value='5050',
        description='Output data WebSocket port'
    )
    
    pointcloud_port_arg = DeclareLaunchArgument(
        'pointcloud_port',
        default_value='5051',
        description='Point cloud WebSocket port'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='sensr',
        description='Frame ID for sensor data'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    # SENSR driver node
    sensr_driver_node = Node(
        package='sensr_ros2_driver',
        executable='sensr_driver_node.py',
        name='sensr_driver',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'output_port': LaunchConfiguration('output_port'),
            'pointcloud_port': LaunchConfiguration('pointcloud_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    return LaunchDescription([
        host_arg,
        output_port_arg,
        pointcloud_port_arg,
        frame_id_arg,
        publish_rate_arg,
        sensr_driver_node,
    ])