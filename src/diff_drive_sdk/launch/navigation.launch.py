#!/usr/bin/env python3
"""
导航系统启动文件
=============

启动差速驱动机器人的导航系统。
"""

import os 
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ... [启动配置和节点定义] ...