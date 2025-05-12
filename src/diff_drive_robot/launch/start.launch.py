#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = '/home/cat/ros2_ws/src/diff_drive_robot'
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_config.yaml')
    
    return LaunchDescription([
        # 启动硬件接口
        Node(
            package='diff_drive_robot',
            executable='hardware_interface',
            name='diff_drive_hardware_interface',
            output='screen'
        ),
        # 启动轮式里程计
        Node(
            package='diff_drive_robot',
            executable='wheel_odom',
            name='wheel_odometry_node',
            output='screen'
        ),
        # 启动EKF节点(带调试参数)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'debug': True, 'debug_out_file': '/tmp/ekf_debug.txt'}
            ]
        ),
    
    ])