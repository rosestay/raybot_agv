#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 获取包目录
    pkg_share = FindPackageShare(package='diff_drive_robot').find('diff_drive_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'diff_drive_robot.urdf')
    
    # 设置URDF描述参数
    robot_description_content = Command(['cat ', urdf_path])
    robot_description = {'robot_description': robot_description_content}
    
    # 机器人状态发布器节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # 关节状态发布器节点
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 关节状态发布器GUI节点
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # 固定帧变换发布器
    fixed_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fixed_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']
    )
    
    # 雷达帧变换发布器 - 确保雷达数据显示在正确位置
    lidar_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_publisher',
        arguments=['0.2', '-0.1', '0', '0', '0', '0', 'lidar_link', 'laser']
    )
    
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
             # Increase publisher queue size
            'qos_overrides.publisher.depth': 100,
            # Increase subscriber queue size if needed
            'qos_overrides.subscription.depth': 100,
            # Enable message batching for higher throughput
            'use_sim_time': False,
            # Additional RMW-specific parameters for queue handling
            'rmw_implementation_specific_qos': True,
        }]
    )
    
 

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时钟'),
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        fixed_frame_publisher,
        lidar_frame_publisher,
        rviz2,
    ])