#!/usr/bin/env python3
"""
ROS2节点实现
==========

处理与ROS2系统的所有交互。
"""

import time
import math
import threading
import sqlite3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

# 导入标准ROS2消息类型
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

# 导入自定义消息类型
from diff_drive_msgs.msg import WheelCommand, Feedback
from diff_drive_msgs.action import NavigateToPoint

# 导入Nav2导航器
from nav2_simple_commander.robot_navigator import BasicNavigator

class RobotNode(Node):
    """ROS2节点类 - 处理与ROS2系统的所有交互"""
    
    def __init__(self, node_name, data_dir):
        """初始化ROS2节点"""
        super().__init__(node_name)
        
        # 数据目录
        self.data_dir = data_dir
        self.db_path = os.path.join(data_dir, 'nav_point.db')
        
        # 数据库锁
        self.db_lock = threading.Lock()
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建发布者
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._wheel_cmd_pub = self.create_publisher(WheelCommand, '/wheel_cmd', 10)
        
        # 创建订阅者
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self._wheel_feedback_sub = self.create_subscription(
            Feedback, '/wheel_feedback', self._wheel_feedback_callback, 10)
        
        # 创建Action客户端
        self._navigate_client = ActionClient(
            self, NavigateToPoint, 'nav_to_point', callback_group=self.callback_group)
        self._current_goal_handle = None
        
        # 创建服务客户端
        self._init_pose_client = self.create_client(
            Trigger, '/init_point', callback_group=self.callback_group)
        self._rem_point_client = self.create_client(
            Trigger, '/rem_point', callback_group=self.callback_group)
        
        # 创建TF监听器
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        
        # 初始化导航器
        self._navigator = None
        
        # 初始化状态变量
        self._pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._velocity = {'linear': 0.0, 'angular': 0.0}
        self._wheel_feedback = {
            'left_position': 0.0,
            'left_velocity': 0.0,
            'right_position': 0.0,
            'right_velocity': 0.0,
            'left_multiturn': 0.0,
            'right_multiturn': 0.0
        }
        self._lock = threading.Lock()
        
        # 创建数据库表
        self.create_nav_point_table()
        
        self.get_logger().info("ROS2节点已初始化")
    
    # ... [类的剩余部分包含上面提到的所有方法] ...