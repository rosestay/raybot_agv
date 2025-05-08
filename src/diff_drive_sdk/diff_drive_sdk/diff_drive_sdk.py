#!/usr/bin/env python3
"""
差速驱动机器人SDK (整合版)
========================

完整的差速驱动机器人SDK，整合了以下功能：
- 基本运动控制
- 轮子反馈监控
- 导航功能
- 导航点存储与管理
- 位置初始化

包含对自定义消息类型和Action的完整支持。
"""

import time
import math
import threading
import os

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .robot_node import RobotNode

class DiffDriveRobot:
    """机器人控制类 - 集成所有功能的主类"""
    
    def __init__(self, node_name="robot_sdk_node", data_dir=None):
        """初始化SDK
        
        参数:
            node_name (str): ROS2节点名称
            data_dir (str): 数据目录路径，用于存储导航点数据库
        """
        # 初始化ROS2上下文
        if not rclpy.ok():
            rclpy.init()
        
        # 设置数据目录
        if data_dir is None:
            # 使用默认路径
            home_dir = os.path.expanduser("~")
            self.data_dir = os.path.join(home_dir, "ros2_ws", "src", "bot_application", "data")
        else:
            self.data_dir = data_dir
        
        # 确保数据目录存在
        os.makedirs(self.data_dir, exist_ok=True)
        
        # 创建ROS2节点
        self._node = RobotNode(node_name, self.data_dir)
        
        # 启动ROS2执行器
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        
        # 等待连接建立
        time.sleep(1.0)
        
        print("差速驱动机器人SDK已初始化")
    
    # ===== 基本移动控制方法 =====
    
    def move(self, linear_speed, angular_speed):
        """控制机器人移动
        
        参数:
            linear_speed (float): 线速度 (m/s)
            angular_speed (float): 角速度 (rad/s)
        """
        self._node.publish_velocity(linear_speed, angular_speed)
    
    def direct_wheel_control(self, left_velocity, right_velocity):
        """直接控制左右轮速度
        
        参数:
            left_velocity (float): 左轮速度 (rad/s)
            right_velocity (float): 右轮速度 (rad/s)
        """
        self._node.publish_wheel_command(left_velocity, right_velocity)
    
    def move_forward(self, speed=0.5):
        """前进
        
        参数:
            speed (float): 速度 (m/s)
        """
        self.move(speed, 0.0)
    
    def move_backward(self, speed=0.5):
        """后退
        
        参数:
            speed (float): 速度 (m/s)
        """
        self.move(-speed, 0.0)
    
    def turn_left(self, speed=0.5):
        """左转
        
        参数:
            speed (float): 转向速度 (rad/s)
        """
        self.move(0.0, speed)
    
    def turn_right(self, speed=0.5):
        """右转
        
        参数:
            speed (float): 转向速度 (rad/s)
        """
        self.move(0.0, -speed)
    
    def stop(self):
        """停止机器人"""
        self.move(0.0, 0.0)
    
    # ===== 位置和反馈方法 =====
    
    def get_position(self):
        """获取当前位置
        
        返回:
            dict: 包含'x', 'y', 'theta'的位置信息
        """
        return self._node.get_current_pose()
    
    def get_velocity(self):
        """获取当前速度
        
        返回:
            dict: 包含'linear', 'angular'的速度信息
        """
        return self._node.get_current_velocity()
    
    def get_wheel_feedback(self):
        """获取轮子反馈信息
        
        返回:
            dict: 包含轮子位置和速度信息
        """
        return self._node.get_wheel_feedback()
    
    # ===== 导航功能 =====
    
    def navigate_to_point(self, point_id, wait=True):
        """导航到指定的点位
        
        参数:
            point_id (int): 导航点位的ID
            wait (bool): 是否等待导航完成
            
        返回:
            dict: 包含导航结果信息
        """
        # 使用action客户端发送导航请求
        return self._node.send_navigate_goal(point_id, wait)
    
    def cancel_navigation(self):
        """取消当前导航任务"""
        return self._node.cancel_navigate_goal()
    
    def move_to(self, x, y, tolerance=0.1, max_speed=0.5):
        """移动到指定位置
        
        参数:
            x (float): 目标X坐标
            y (float): 目标Y坐标
            tolerance (float): 位置容差
            max_speed (float): 最大速度
            
        返回:
            bool: 是否到达目标位置
        """
        return self._node.move_to_position(x, y, tolerance, max_speed)
    
    def rotate_to(self, angle, tolerance=0.1, speed=0.5):
        """旋转到指定角度
        
        参数:
            angle (float): 目标角度(弧度)
            tolerance (float): 角度容差
            speed (float): 旋转速度
            
        返回:
            bool: 是否到达目标角度
        """
        # 规范化角度到[-π, π]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        
        start_time = time.time()
        timeout = 10.0  # 10秒超时
        
        while True:
            if time.time() - start_time > timeout:
                print("旋转到目标角度超时")
                self.stop()
                return False
            
            # 获取当前位置
            current_pos = self.get_position()
            current_angle = current_pos['theta']
            
            # 计算角度差
            angle_diff = angle - current_angle
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # 检查是否达到目标角度
            if abs(angle_diff) < tolerance:
                self.stop()
                return True
            
            # 确定旋转方向和速度
            angular_speed = min(speed, 0.5 + 0.5 * abs(angle_diff))
            if angle_diff > 0:
                self.move(0.0, angular_speed)
            else:
                self.move(0.0, -angular_speed)
            
            time.sleep(0.1)
    
    # ===== 导航点管理 =====
    
    def remember_current_point(self):
        """记忆当前位置作为导航点
        
        返回:
            dict: 包含操作结果信息
        """
        return self._node.remember_point()
    
    def init_robot_pose(self):
        """初始化机器人位姿
        
        返回:
            dict: 包含操作结果信息
        """
        return self._node.init_robot_pose()
    
    def get_all_nav_points(self):
        """获取所有导航点
        
        返回:
            list: 导航点列表
        """
        return self._node.get_all_nav_points()
    
    def follow_waypoints(self, waypoint_ids):
        """按顺序导航到多个点位
        
        参数:
            waypoint_ids (list): 导航点ID列表
            
        返回:
            bool: 导航是否成功完成
        """
        return self._node.follow_waypoints(waypoint_ids)
    
    # ===== 资源管理 =====
    
    def shutdown(self):
        """关闭SDK并释放资源"""
        self.stop()
        if hasattr(self, '_thread') and self._thread.is_alive():
            self._node.cleanup()
            rclpy.shutdown()
            self._thread.join(timeout=1.0)
        print("SDK已关闭")