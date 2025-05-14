#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy as np
from threading import Lock
import yaml
import cv2
import os
from PIL import Image

class MapFusionNode(Node):
    def __init__(self):
        super().__init__('map_fusion_node')
        
        # 参数
        self.declare_parameter('base_map_file', '')  # 基础地图文件路径
        self.declare_parameter('fusion_method', 'weighted')  # 融合方法: weighted, max, min
        self.declare_parameter('base_map_weight', 0.7)  # 基础地图权重
        self.declare_parameter('slam_map_weight', 0.3)  # SLAM地图权重
        self.declare_parameter('update_threshold', 0.05)  # 更新阈值
        self.declare_parameter('publish_rate', 1.0)  # 发布频率
        
        # 获取参数
        self.base_map_file = self.get_parameter('base_map_file').value
        self.fusion_method = self.get_parameter('fusion_method').value
        self.base_map_weight = self.get_parameter('base_map_weight').value
        self.slam_map_weight = self.get_parameter('slam_map_weight').value
        self.update_threshold = self.get_parameter('update_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 订阅SLAM地图
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.slam_map_callback,
            10
        )
        
        # 发布融合地图
        self.fused_map_pub = self.create_publisher(
            OccupancyGrid,
            '/fused_map',
            10
        )
        
        # 数据存储
        self.base_map = None
        self.slam_map = None
        self.fused_map = None
        self.map_lock = Lock()
        
        # 加载基础地图
        self.load_base_map()
        
        # 定时器
        self.fusion_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.fusion_timer_callback
        )
        
        self.get_logger().info(f'地图融合节点启动，融合方法: {self.fusion_method}')
        
    def load_base_map(self):
        """加载基础地图"""
        if not self.base_map_file:
            self.get_logger().warn('未指定基础地图文件')
            return
            
        try:
            # 加载yaml文件
            yaml_file = self.base_map_file
            if not yaml_file.endswith('.yaml'):
                yaml_file = yaml_file.replace('.pgm', '.yaml')
                
            with open(yaml_file, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            # 加载地图图像
            image_file = os.path.join(
                os.path.dirname(yaml_file),
                map_metadata['image']
            )
            
            # 使用PIL加载图像
            img = Image.open(image_file)
            map_data = np.array(img)
            
            # 创建OccupancyGrid消息
            self.base_map = OccupancyGrid()
            self.base_map.header.frame_id = 'map'
            self.base_map.info.resolution = map_metadata['resolution']
            self.base_map.info.width = map_data.shape[1]
            self.base_map.info.height = map_data.shape[0]
            
            # 设置原点
            origin = map_metadata['origin']
            self.base_map.info.origin.position.x = origin[0]
            self.base_map.info.origin.position.y = origin[1]
            self.base_map.info.origin.position.z = 0.0
            self.base_map.info.origin.orientation.w = 1.0
            
            # 转换地图数据格式
            # PGM: 0=black=occupied, 255=white=free, 其他=unknown
            occupancy_data = []
            for pixel in map_data.flatten():
                if pixel == 0:  # 黑色 - 占用
                    occupancy_data.append(100)
                elif pixel == 255:  # 白色 - 空闲
                    occupancy_data.append(0)
                else:  # 灰色 - 未知
                    occupancy_data.append(-1)
                    
            self.base_map.data = occupancy_data
            
            self.get_logger().info(f'成功加载基础地图: {yaml_file}')
            
        except Exception as e:
            self.get_logger().error(f'加载基础地图失败: {str(e)}')
            
    def slam_map_callback(self, msg):
        """接收SLAM地图"""
        with self.map_lock:
            self.slam_map = msg
            
    def fusion_timer_callback(self):
        """定时执行地图融合"""
        with self.map_lock:
            if self.slam_map is None:
                if self.base_map is not None:
                    # 如果只有基础地图，发布基础地图
                    self.fused_map_pub.publish(self.base_map)
                return
                
            if self.base_map is None:
                # 如果只有SLAM地图，发布SLAM地图
                self.fused_map_pub.publish(self.slam_map)
                return
                
            # 执行地图融合
            self.fuse_maps()
            
            if self.fused_map is not None:
                self.fused_map_pub.publish(self.fused_map)
                
    def fuse_maps(self):
        """融合地图"""
        try:
            # 确保地图大小和分辨率匹配
            if not self.check_map_compatibility():
                self.get_logger().warn('地图不兼容，尝试对齐')
                self.align_maps()
                
            # 根据不同的融合方法进行融合
            if self.fusion_method == 'weighted':
                self.weighted_fusion()
            elif self.fusion_method == 'max':
                self.max_fusion()
            elif self.fusion_method == 'min':
                self.min_fusion()
            else:
                self.get_logger().error(f'未知的融合方法: {self.fusion_method}')
                
        except Exception as e:
            self.get_logger().error(f'地图融合失败: {str(e)}')
            
    def check_map_compatibility(self):
        """检查地图兼容性"""
        if (self.base_map.info.width != self.slam_map.info.width or
            self.base_map.info.height != self.slam_map.info.height or
            abs(self.base_map.info.resolution - self.slam_map.info.resolution) > 1e-6):
            return False
        return True
        
    def align_maps(self):
        """对齐地图（简化版本，实际应用中可能需要更复杂的对齐算法）"""
        # 这里简化处理，只考虑分辨率相同的情况
        if abs(self.base_map.info.resolution - self.slam_map.info.resolution) > 1e-6:
            self.get_logger().error('地图分辨率不同，无法对齐')
            return
            
        # TODO: 实现更复杂的地图对齐算法
        self.get_logger().warn('地图对齐功能尚未完全实现')
        
    def weighted_fusion(self):
        """加权融合"""
        # 创建融合地图
        self.fused_map = OccupancyGrid()
        self.fused_map.header = self.slam_map.header
        self.fused_map.info = self.slam_map.info
        
        # 将地图数据转换为numpy数组
        base_data = np.array(self.base_map.data, dtype=np.float32)
        slam_data = np.array(self.slam_map.data, dtype=np.float32)
        
        # 处理未知区域
        base_known = base_data >= 0
        slam_known = slam_data >= 0
        
        # 初始化融合数据
        fused_data = np.full_like(base_data, -1)
        
        # 两个地图都已知的区域：加权平均
        both_known = base_known & slam_known
        fused_data[both_known] = (
            self.base_map_weight * base_data[both_known] +
            self.slam_map_weight * slam_data[both_known]
        )
        
        # 只有基础地图已知的区域：使用基础地图，但降低权重
        base_only = base_known & ~slam_known
        fused_data[base_only] = base_data[base_only] * 0.8
        
        # 只有SLAM地图已知的区域：使用SLAM地图
        slam_only = ~base_known & slam_known
        fused_data[slam_only] = slam_data[slam_only]
        
        # 转换回整数
        fused_data = np.clip(fused_data, -1, 100)
        fused_data[fused_data >= 0] = np.round(fused_data[fused_data >= 0])
        
        self.fused_map.data = fused_data.astype(np.int8).tolist()
        
    def max_fusion(self):
        """最大值融合（更相信占用）"""
        self.fused_map = OccupancyGrid()
        self.fused_map.header = self.slam_map.header
        self.fused_map.info = self.slam_map.info
        
        base_data = np.array(self.base_map.data)
        slam_data = np.array(self.slam_map.data)
        
        # 对于已知区域，取最大值（更相信占用）
        fused_data = np.maximum(base_data, slam_data)
        
        self.fused_map.data = fused_data.tolist()
        
    def min_fusion(self):
        """最小值融合（更相信空闲）"""
        self.fused_map = OccupancyGrid()
        self.fused_map.header = self.slam_map.header
        self.fused_map.info = self.slam_map.info
        
        base_data = np.array(self.base_map.data)
        slam_data = np.array(self.slam_map.data)
        
        # 处理未知区域
        base_data_adj = base_data.copy()
        slam_data_adj = slam_data.copy()
        
        # 将未知区域设为最大值，这样在取最小值时会被忽略
        base_data_adj[base_data < 0] = 101
        slam_data_adj[slam_data < 0] = 101
        
        # 取最小值
        fused_data = np.minimum(base_data_adj, slam_data_adj)
        
        # 恢复未知区域
        both_unknown = (base_data < 0) & (slam_data < 0)
        fused_data[both_unknown] = -1
        fused_data[fused_data > 100] = -1
        
        self.fused_map.data = fused_data.tolist()

def main(args=None):
    rclpy.init(args=args)
    
    fusion_node = MapFusionNode()
    
    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()