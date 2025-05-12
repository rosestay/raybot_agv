#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg
import os

def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = '/home/cat/ros2_ws/src/diff_drive_robot'
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_config.yaml')
    
    # 获取激光雷达配置文件路径
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params','lidar_uart_ros2', 'lsn10p.yaml')
    
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
        
        # 启动EKF节点
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
        
        # 启动IMU节点
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu',
            remappings=[('/wit/imu', '/imu/data')],
            parameters=[{'port': '/dev/imu_usb'},
                       {"baud": 9600}],
            output="screen"
        ),
        
        # 启动激光雷达节点
        LifecycleNode(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',		#设置激光数据topic名称
            output='screen',
            emulate_tty=True,
            namespace='',
            parameters=[driver_dir],
        ),
        
        # 启动RViz节点（已注释，根据需要启用）
        # Node(
        #     package='rviz2',
        #     executable="rviz2",
        #     output="screen"
        # ),
    ])