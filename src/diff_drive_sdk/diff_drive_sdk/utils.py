#!/usr/bin/env python3
"""
工具函数
=======

提供SDK使用的各种辅助函数。
"""

import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数
    
    参数:
        roll (float): 横滚角(弧度)
        pitch (float): 俯仰角(弧度)
        yaw (float): 偏航角(弧度)
        
    返回:
        geometry_msgs.msg.Quaternion: 四元数消息
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    return q

def quaternion_to_euler(q):
    """四元数转欧拉角
    
    参数:
        q (geometry_msgs.msg.Quaternion): 四元数消息
        
    返回:
        tuple: (roll, pitch, yaw)三个欧拉角(弧度)
    """
    # 提取四元数分量
    x, y, z, w = q.x, q.y, q.z, q.w
    
    # 转换为欧拉角
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

def normalize_angle(angle):
    """将角度规范化到[-pi, pi]范围
    
    参数:
        angle (float): 输入角度(弧度)
        
    返回:
        float: 规范化后的角度(弧度)
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle