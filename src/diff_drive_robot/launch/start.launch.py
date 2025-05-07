#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_robot',
            executable='hardware_interface',
            name='diff_drive_hardware_interface',
            output='screen'
        ),
        Node(
            package='diff_drive_robot',
            executable='wheel_odom',
            name='wheel_odometry_node',
            output='screen'
        )
    ])