#!/usr/bin/env python3
"""
导航到点位Action服务器
==================

处理导航到预设点位的Action请求。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import sqlite3
import os
import threading
import time
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from diff_drive_msgs.action import NavigateToPoint

# ... [类定义与实现] ...