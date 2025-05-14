#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy as np
import os
from datetime import datetime

class SlamMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # 参数
        self.declare_parameter('map_change_threshold', 0.1)  # 10%的变化触发警告
        self.declare_parameter('pose_uncertainty_threshold', 0.5)  # 位置不确定性阈值
        
        self.map_change_threshold = self.get_parameter('map_change_threshold').value
        self.pose_uncertainty_threshold = self.get_parameter('pose_uncertainty_threshold').value
        
        # 订阅
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )
        
        # 发布状态
        self.status_pub = self.create_publisher(String, '/slam_status', 10)
        
        # 状态变量
        self.last_map = None
        self.map_changes = []
        self.pose_uncertainties = []
        
        # 定时器
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('SLAM监控节点启动')
        
    def map_callback(self, msg):
        if self.last_map is not None:
            # 计算地图变化
            change_ratio = self.calculate_map_change(self.last_map, msg)
            self.map_changes.append(change_ratio)
            
            if change_ratio > self.map_change_threshold:
                self.get_logger().warn(f'地图变化较大: {change_ratio:.2%}')
                
        self.last_map = msg
        
    def pose_callback(self, msg):
        # 计算位置不确定性
        covariance = msg.pose.covariance
        uncertainty = np.sqrt(covariance[0] + covariance[7])  # x和y方向的不确定性
        self.pose_uncertainties.append(uncertainty)
        
        if uncertainty > self.pose_uncertainty_threshold:
            self.get_logger().warn(f'定位不确定性较高: {uncertainty:.3f}')
            
    def calculate_map_change(self, old_map, new_map):
        # 将地图数据转换为numpy数组
        old_data = np.array(old_map.data)
        new_data = np.array(new_map.data)
        
        # 只比较已知区域（非未知区域）
        known_mask = (old_data >= 0) & (new_data >= 0)
        
        if np.sum(known_mask) == 0:
            return 0.0
            
        # 计算变化的像素数
        changed_pixels = np.sum(old_data[known_mask] != new_data[known_mask])
        change_ratio = changed_pixels / np.sum(known_mask)
        
        return change_ratio
        
    def publish_status(self):
        status = String()
        
        # 计算平均指标
        avg_map_change = np.mean(self.map_changes) if self.map_changes else 0.0
        avg_uncertainty = np.mean(self.pose_uncertainties) if self.pose_uncertainties else 0.0
        
        status_dict = {
            'timestamp': datetime.now().isoformat(),
            'avg_map_change': f'{avg_map_change:.2%}',
            'avg_pose_uncertainty': f'{avg_uncertainty:.3f}',
            'map_updates': len(self.map_changes),
            'status': 'OK'
        }
        
        # 判断状态
        if avg_map_change > self.map_change_threshold:
            status_dict['status'] = 'MAP_CHANGING'
        if avg_uncertainty > self.pose_uncertainty_threshold:
            status_dict['status'] = 'HIGH_UNCERTAINTY'
            
        status.data = str(status_dict)
        self.status_pub.publish(status)
        
        # 清空历史数据（保留最近的数据）
        self.map_changes = self.map_changes[-10:] if len(self.map_changes) > 10 else self.map_changes
        self.pose_uncertainties = self.pose_uncertainties[-10:] if len(self.pose_uncertainties) > 10 else self.pose_uncertainties

def main(args=None):
    rclpy.init(args=args)
    
    monitor = SlamMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()