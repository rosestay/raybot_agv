#!/usr/bin/env python3
# realtime_map_updater.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from threading import Lock

class RealtimeMapUpdater(Node):
    def __init__(self):
        super().__init__('realtime_map_updater')
        
        # 参数
        self.declare_parameter('update_threshold', 0.10)  
        self.declare_parameter('min_update_interval', 1.0)  # 最小更新间隔
        
        self.update_threshold = self.get_parameter('update_threshold').value
        self.min_update_interval = self.get_parameter('min_update_interval').value
        
        # 订阅地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # 发布更新的地图
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/updated_map',
            10
        )
        
        # 状态变量
        self.current_map = None
        self.last_published_map = None
        self.last_update_time = self.get_clock().now()
        self.map_lock = Lock()
        
        # 定时器用于定期检查更新
        self.update_timer = self.create_timer(0.1, self.check_and_update_map)
        
        self.get_logger().info('实时地图更新节点启动')
        
    def map_callback(self, msg):
        with self.map_lock:
            self.current_map = msg
            
    def check_and_update_map(self):
        with self.map_lock:
            if self.current_map is None:
                return
                
            current_time = self.get_clock().now()
            time_since_last_update = (current_time - self.last_update_time).nanoseconds / 1e9
            
            # 检查时间间隔
            if time_since_last_update < self.min_update_interval:
                return
                
            # 检查地图变化
            if self.last_published_map is not None:
                if not self.map_changed_significantly(self.last_published_map, self.current_map):
                    return
                    
            # 发布更新的地图
            self.map_pub.publish(self.current_map)
            self.last_published_map = self.current_map
            self.last_update_time = current_time
            self.get_logger().info('地图已更新')
            
    def map_changed_significantly(self, old_map, new_map):
        # 比较地图大小
        if (old_map.info.width != new_map.info.width or 
            old_map.info.height != new_map.info.height):
            return True
            
        # 比较地图内容
        old_data = np.array(old_map.data)
        new_data = np.array(new_map.data)
        
        # 只比较已知区域
        known_mask = (old_data >= 0) & (new_data >= 0)
        if np.sum(known_mask) == 0:
            return False
            
        # 计算变化比例
        changed_cells = np.sum(old_data[known_mask] != new_data[known_mask])
        change_ratio = changed_cells / np.sum(known_mask)
        
        return change_ratio > self.update_threshold

def main(args=None):
    rclpy.init(args=args)
    updater = RealtimeMapUpdater()
    
    try:
        rclpy.spin(updater)
    except KeyboardInterrupt:
        pass
    finally:
        updater.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()