#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np

class DynamicMapLoader(Node):
    def __init__(self):
        super().__init__('dynamic_map_loader')
        
        # 参数
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/map')
        self.declare_parameter('publish_rate', 2.0)
        
        self.map_topic = self.get_parameter('map_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 订阅SLAM地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        
        # 发布地图（供导航使用）
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            10
        )
        
        self.current_map = None
        
        # 定时发布地图
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_map)
        
        self.get_logger().info(f'动态地图加载器启动: {self.map_topic} -> {self.output_topic}')
        
    def map_callback(self, msg):
        self.current_map = msg
        self.get_logger().debug('收到新地图')
            
    def publish_map(self):
        if self.current_map is not None:
            # 更新时间戳
            self.current_map.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.current_map)
            self.get_logger().debug('发布地图')
        else:
            # 如果没有地图，发布一个空地图
            empty_map = OccupancyGrid()
            empty_map.header.stamp = self.get_clock().now().to_msg()
            empty_map.header.frame_id = "map"
            empty_map.info.resolution = 0.05
            empty_map.info.width = 100
            empty_map.info.height = 100
            empty_map.info.origin.position.x = -2.5
            empty_map.info.origin.position.y = -2.5
            empty_map.info.origin.position.z = 0.0
            empty_map.info.origin.orientation.w = 1.0
            
            # 创建未知区域地图
            empty_map.data = [-1] * (100 * 100)
            
            self.map_pub.publish(empty_map)
            self.get_logger().debug('发布空地图')

def main(args=None):
    rclpy.init(args=args)
    loader = DynamicMapLoader()
    
    try:
        rclpy.spin(loader)
    except KeyboardInterrupt:
        pass
    finally:
        loader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()