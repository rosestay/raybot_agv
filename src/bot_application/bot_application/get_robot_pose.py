#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
import sqlite3
import math
from datetime import datetime
import os
import threading

# 引入服务接口和回调组
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Quaternion, Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class NavPointStorageNode(Node):
    def __init__(self):
        super().__init__('nav_point_storage_node')
        
        # 创建回调组以提高服务处理效率
        self.callback_group = ReentrantCallbackGroup()
        
        # 使用指定的数据库路径
        data_dir = '/home/cat/ros2_ws/src/bot_application/data'
        # 确保目录存在
        os.makedirs(data_dir, exist_ok=True)
        self.db_path = os.path.join(data_dir, 'nav_point.db')
        self.get_logger().info(f'数据库路径: {self.db_path}')
        
        # 为数据库连接创建一个锁
        self.db_lock = threading.Lock()
        
        # 初始化数据库连接 - 不在初始化时连接，而是在需要时连接
        self.create_nav_point_table()
        
        # TF设置
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        
        # 创建ROS服务
        self.srv = self.create_service(
            Trigger, 
            '/rem_point', 
            self.handle_rem_point,
            callback_group=self.callback_group
        )
        self.get_logger().info('导航点存储节点已初始化，等待服务请求...')

    def get_db_connection(self):
        """获取一个新的数据库连接（线程安全）"""
        return sqlite3.connect(self.db_path)

    def create_nav_point_table(self):
        """创建存储导航点数据的表格（如果不存在）"""
        try:
            with self.db_lock:
                conn = self.get_db_connection()
                cursor = conn.cursor()
                cursor.execute('''
                    CREATE TABLE IF NOT EXISTS nav_points (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp TEXT,
                        translation_x REAL,
                        translation_y REAL,
                        translation_z REAL,
                        rotation_x REAL,
                        rotation_y REAL,
                        rotation_z REAL,
                        rotation_w REAL,
                        roll REAL,
                        pitch REAL,
                        yaw REAL,
                        is_remembered INTEGER DEFAULT 0
                    )
                ''')
                conn.commit()
                conn.close()
                self.get_logger().info('数据库表已创建或已存在')
        except sqlite3.Error as e:
            self.get_logger().error(f'创建表格错误: {str(e)}')
            raise e

    def handle_rem_point(self, request, response):
        """处理记忆导航点的服务回调（线程安全）"""
        self.get_logger().info("收到记忆导航点请求")
        
        try:
            # 查找变换
            result = self.buffer_.lookup_transform(
                'map', 
                'base_footprint',
                rclpy.time.Time(),  # 使用当前时间
                rclpy.time.Duration(seconds=1.0)
            )
            transform = result.transform
            
            # 转换四元数为欧拉角
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            
            # 准备数据库存储
            timestamp = datetime.now().isoformat()
            
            try:
                # 在当前线程中创建新的数据库连接
                with self.db_lock:
                    conn = self.get_db_connection()
                    cursor = conn.cursor()
                    cursor.execute('''
                        INSERT INTO nav_points 
                        (timestamp, 
                        translation_x, translation_y, translation_z, 
                        rotation_x, rotation_y, rotation_z, rotation_w,
                        roll, pitch, yaw,
                        is_remembered) 
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ''', (
                        timestamp,
                        float(transform.translation.x), 
                        float(transform.translation.y), 
                        float(transform.translation.z),
                        float(transform.rotation.x), 
                        float(transform.rotation.y), 
                        float(transform.rotation.z), 
                        float(transform.rotation.w),
                        float(rotation_euler[0]), 
                        float(rotation_euler[1]), 
                        float(rotation_euler[2]),
                        1  # 直接标记为已记忆
                    ))
                    conn.commit()
                    
                    # 验证数据已被写入
                    cursor.execute("SELECT last_insert_rowid()")
                    point_id = cursor.fetchone()[0]
                    
                    # 关闭连接
                    conn.close()
                    
                    # 准备响应
                    response.success = True
                    response.message = f"成功记忆导航点 (ID: {point_id})"
                    
                    self.get_logger().info(f"记忆当前导航点成功: ID={point_id}, x={transform.translation.x}, y={transform.translation.y}")
                
            except sqlite3.Error as db_err:
                response.success = False
                response.message = f"数据库写入错误: {str(db_err)}"
                self.get_logger().error(f"数据库写入错误: {str(db_err)}")
                
        except Exception as e:
            response.success = False
            response.message = f"记忆导航点时发生错误: {str(e)}"
            self.get_logger().error(f"记忆导航点失败: {str(e)}")
            
        return response

def main():
    """主函数"""
    rclpy.init()
    
    # 创建节点
    node = NavPointStorageNode()
    
    # 使用多线程执行器以提高性能
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info('开始运行导航点存储节点...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在关闭节点...')
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()