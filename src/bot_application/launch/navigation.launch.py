#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人导航启动文件
用于同时启动初始化位姿、记忆导航点和导航到指定点位的节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间 (true/false)'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='自动启动导航系统 (true/false)'
    )
    
    # 获取脚本文件的路径 - 使用FindPackageShare更加可靠
    scripts_path = PathJoinSubstitution([FindPackageShare('bot_application'), 'scripts'])
    
    # 1. 初始化位姿节点 - 使用节点可执行文件
    init_pose_node = Node(
        package='bot_application',
        executable='init_robot_pose',
        name='nav2_init_point_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 2. 记忆导航点节点 - 使用节点可执行文件
    storage_node = Node(
        package='bot_application',
        executable='get_robot_pose',
        name='nav_point_storage_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 3. 导航到点位节点 - 使用节点可执行文件
    nav_to_pose_node = Node(
        package='bot_application',
        executable='nav_to_pose',
        name='nav_to_point_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 当导航到点位节点启动后，自动初始化机器人位姿
    auto_init_pose = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=nav_to_pose_node,
            on_start=[
                LogInfo(msg='导航到点位节点已启动，正在初始化机器人位姿...'),
                ExecuteProcess(
                    cmd=[[
                        FindExecutable(name='ros2'),
                        ' service call ',
                        '/init_point ',
                        'std_srvs/srv/Trigger ',
                        '{}'
                    ]],
                    shell=True,
                    condition=IfCondition(autostart)
                )
            ]
        )
    )
    
    # 返回启动描述
    return LaunchDescription([
        # 声明参数
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        
        # 启动节点
        init_pose_node,
        storage_node,
        nav_to_pose_node,
        
        # 自动初始化位姿（如果开启了autostart）
        auto_init_pose
    ])