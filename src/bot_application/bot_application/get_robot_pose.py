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

# 创建自定义服务接口
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Quaternion, Pose

class NavPointStorageNode(Node):
    def __init__(self):
        super().__init__('nav_point_storage_node')
        
        # 使用指定的数据库路径
        data_dir = '/home/cat/ros2_ws/src/bot_application/data'
        # 确保目录存在
        os.makedirs(data_dir, exist_ok=True)
        self.db_path = os.path.join(data_dir, 'nav_point.db')
        self.get_logger().info(f'数据库路径: {self.db_path}')
        
        # 初始化数据库连接
        try:
            self.conn = sqlite3.connect(self.db_path)
            self.create_nav_point_table()
            self.get_logger().info('成功连接到数据库')
        except sqlite3.Error as e:
            self.get_logger().error(f'数据库连接错误: {str(e)}')
            raise e
        
        # TF设置
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        
        # 删除定时器，不自动存储位置
        # self.timer_ = self.create_timer(storage_freq, self.store_nav_point)
        
        # 创建ROS服务
        self.srv = self.create_service(Trigger, '/rem_point', self.handle_rem_point)
        self.get_logger().info('导航点存储节点已初始化')

    def create_nav_point_table(self):
        """创建存储导航点数据的表格（如果不存在）"""
        try:
            cursor = self.conn.cursor()
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
            self.conn.commit()
            self.get_logger().info('数据库表已创建或已存在')
        except sqlite3.Error as e:
            self.get_logger().error(f'创建表格错误: {str(e)}')
            raise e

    def store_nav_point(self):
        """存储当前导航点到数据库"""
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
                cursor = self.conn.cursor()
                cursor.execute('''
                    INSERT INTO nav_points 
                    (timestamp, 
                    translation_x, translation_y, translation_z, 
                    rotation_x, rotation_y, rotation_z, rotation_w,
                    roll, pitch, yaw) 
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
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
                    float(rotation_euler[2])
                ))
                self.conn.commit()
                
                # 验证数据已被写入
                cursor.execute("SELECT last_insert_rowid()")
                last_id = cursor.fetchone()[0]
                
                # 记录日志
                self.get_logger().info(f'成功存储导航点 (ID: {last_id}) 于 {timestamp}')
                self.get_logger().debug(f'平移: x={transform.translation.x}, y={transform.translation.y}, z={transform.translation.z}')
                self.get_logger().debug(f'旋转 (四元数): x={transform.rotation.x}, y={transform.rotation.y}, z={transform.rotation.z}, w={transform.rotation.w}')
                self.get_logger().debug(f'旋转 (RPY): roll={rotation_euler[0]}, pitch={rotation_euler[1]}, yaw={rotation_euler[2]}')
                
            except sqlite3.Error as db_err:
                self.get_logger().error(f'数据库写入错误: {str(db_err)}')
                # 尝试重新连接数据库
                self.reconnect_database()
        
        except Exception as e:
            self.get_logger().warn(f'获取和存储导航点失败: {str(e)}')

    def handle_rem_point(self, request, response):
        """处理记忆导航点的服务回调"""
        try:
            # 直接存储当前位置，而不是获取之前记录的点
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
                
                cursor = self.conn.cursor()
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
                self.conn.commit()
                
                # 验证数据已被写入
                cursor.execute("SELECT last_insert_rowid()")
                point_id = cursor.fetchone()[0]
                
                # 准备响应
                response.success = True
                response.message = f"成功记忆导航点 (ID: {point_id})"
                
                self.get_logger().info(f"记忆当前导航点成功: ID={point_id}, x={transform.translation.x}, y={transform.translation.y}")
                
            except sqlite3.Error as db_err:
                response.success = False
                response.message = f"数据库写入错误: {str(db_err)}"
                self.get_logger().error(f"数据库写入错误: {str(db_err)}")
                # 尝试重新连接数据库
                self.reconnect_database()
                
        except Exception as e:
            response.success = False
            response.message = f"记忆导航点时发生错误: {str(e)}"
            self.get_logger().error(f"记忆导航点失败: {str(e)}")
            
        return response

    def reconnect_database(self):
        """重新连接数据库"""
        try:
            if hasattr(self, 'conn'):
                self.conn.close()
            self.conn = sqlite3.connect(self.db_path)
            self.get_logger().info('数据库重新连接成功')
        except sqlite3.Error as e:
            self.get_logger().error(f'数据库重新连接失败: {str(e)}')

    def __del__(self):
        """析构函数，确保关闭数据库连接"""
        try:
            if hasattr(self, 'conn'):
                self.conn.close()
                self.get_logger().info('数据库连接已关闭')
        except Exception as e:
            self.get_logger().error(f'关闭数据库连接时出错: {str(e)}')

def main():
    """主函数"""
    rclpy.init()
    node = NavPointStorageNode()
    
    try:
        node.get_logger().info('开始运行导航点存储节点...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在关闭节点...')
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()