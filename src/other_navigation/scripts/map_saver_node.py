#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
import yaml
import os
from datetime import datetime

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        
        # 参数
        self.declare_parameter('save_interval', 60.0)  # 保存间隔（秒）
        self.declare_parameter('map_save_path', '/home/robot/maps/')
        self.declare_parameter('map_name_prefix', 'dynamic_map')
        
        self.save_interval = self.get_parameter('save_interval').value
        self.map_save_path = self.get_parameter('map_save_path').value
        self.map_name_prefix = self.get_parameter('map_name_prefix').value
        
        # 确保保存路径存在
        os.makedirs(self.map_save_path, exist_ok=True)
        
        # 创建定时器
        self.save_timer = self.create_timer(self.save_interval, self.save_map_callback)
        
        # 创建服务客户端
        self.map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')
        
        # 订阅地图话题
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.current_map = None
        
        self.get_logger().info(f'地图保存节点启动，保存间隔: {self.save_interval}秒')
        
    def map_callback(self, msg):
        self.current_map = msg
        
    def save_map_callback(self):
        if self.current_map is None:
            self.get_logger().warn('还没有收到地图数据，跳过保存')
            return
            
        # 生成时间戳文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_name = f'{self.map_name_prefix}_{timestamp}'
        
        # 创建保存请求
        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = os.path.join(self.map_save_path, map_name)
        request.image_format = 'pgm'
        request.map_mode = SaveMap.Request.TRINARY
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        # 等待服务可用
        while not self.map_saver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待地图保存服务...')
            
        # 发送请求
        future = self.map_saver_client.call_async(request)
        future.add_done_callback(lambda f: self.save_map_response(f, map_name))
        
    def save_map_response(self, future, map_name):
        try:
            response = future.result()
            if response.result:
                self.get_logger().info(f'地图成功保存: {map_name}')
                
                # 创建符号链接指向最新的地图
                latest_link = os.path.join(self.map_save_path, 'latest_map.pgm')
                map_file = f'{map_name}.pgm'
                map_full_path = os.path.join(self.map_save_path, map_file)
                
                if os.path.exists(latest_link):
                    os.remove(latest_link)
                os.symlink(map_full_path, latest_link)
                
                # 同时更新yaml文件的链接
                latest_yaml_link = os.path.join(self.map_save_path, 'latest_map.yaml')
                yaml_file = f'{map_name}.yaml'
                yaml_full_path = os.path.join(self.map_save_path, yaml_file)
                
                if os.path.exists(latest_yaml_link):
                    os.remove(latest_yaml_link)
                os.symlink(yaml_full_path, latest_yaml_link)
                
            else:
                self.get_logger().error(f'保存地图失败: {map_name}')
        except Exception as e:
            self.get_logger().error(f'保存地图时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    map_saver = MapSaverNode()
    
    try:
        rclpy.spin(map_saver)
    except KeyboardInterrupt:
        pass
    finally:
        map_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()