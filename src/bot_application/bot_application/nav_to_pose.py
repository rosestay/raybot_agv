#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import sqlite3
import os
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler
import threading
import sys

class NavToPointNode(Node):
    def __init__(self):
        super().__init__('nav_to_point_node')
        
        # 数据库路径设置
        data_dir = '/home/cat/ros2_ws/src/bot_application/data'
        self.db_path = os.path.join(data_dir, 'nav_point.db')
        self.get_logger().info(f'使用数据库路径: {self.db_path}')
        
        # 检查数据库是否存在
        if not os.path.exists(self.db_path):
            self.get_logger().error(f'数据库文件不存在: {self.db_path}')
            raise FileNotFoundError(f'数据库文件不存在: {self.db_path}')
        
        # 初始化数据库连接
        try:
            self.conn = sqlite3.connect(self.db_path)
            self.get_logger().info('成功连接到数据库')
        except sqlite3.Error as e:
            self.get_logger().error(f'数据库连接错误: {str(e)}')
            raise e
        
        # 初始化导航器
        self.navigator = BasicNavigator()
        self.get_logger().info('等待Nav2系统激活...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2系统已激活')
        
        # 创建导航服务
        self.srv = self.create_service(Trigger, '/nav_to_point', self.handle_nav_to_point)
        self.get_logger().info('导航到点位服务已创建')
        
        # 当前导航的点位ID
        self.current_nav_id = None
        
        # 定时器用于检查导航状态
        self.nav_timer = None
        
        # 启动键盘输入线程
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        self.get_logger().info('键盘输入线程已启动，请输入导航点ID')
    
    def keyboard_input_loop(self):
        """处理键盘输入的线程"""
        while self.running:
            try:
                print("\n请输入要导航的点位ID (输入'q'退出): ")
                user_input = input().strip()
                
                if user_input.lower() == 'q':
                    self.get_logger().info('收到退出命令')
                    self.running = False
                    rclpy.shutdown()
                    break
                
                if user_input.lower() == 'c':
                    # 取消当前导航
                    if self.cancel_navigation():
                        print("导航已取消")
                    else:
                        print("没有正在进行的导航")
                    continue
                
                try:
                    point_id = int(user_input)
                    self.navigate_to_point_by_id(point_id)
                except ValueError:
                    print("请输入有效的点位ID（整数）")
            except Exception as e:
                self.get_logger().error(f'键盘输入处理错误: {str(e)}')
    
    def navigate_to_point_by_id(self, point_id):
        """根据点位ID导航到指定位置"""
        try:
            # 从数据库查询指定ID的点位
            cursor = self.conn.cursor()
            cursor.execute('''
                SELECT id, translation_x, translation_y, translation_z, 
                       rotation_x, rotation_y, rotation_z, rotation_w,
                       roll, pitch, yaw
                FROM nav_points 
                WHERE id = ?
            ''', (point_id,))
            point = cursor.fetchone()
            
            if point:
                point_id, x, y, z, qx, qy, qz, qw, roll, pitch, yaw = point
                
                # 创建目标姿态
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "map"
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                
                # 设置位置
                goal_pose.pose.position.x = float(x)
                goal_pose.pose.position.y = float(y)
                goal_pose.pose.position.z = float(z)
                
                # 设置方向
                goal_pose.pose.orientation.x = float(qx)
                goal_pose.pose.orientation.y = float(qy)
                goal_pose.pose.orientation.z = float(qz)
                goal_pose.pose.orientation.w = float(qw)
                
                # 开始导航
                self.get_logger().info(f'开始导航到点位 (ID: {point_id}): x={x}, y={y}, z={z}')
                self.navigator.goToPose(goal_pose)
                
                # 记录当前导航的点位ID
                self.current_nav_id = point_id
                
                # 启动定时器监控导航状态
                if self.nav_timer is not None:
                    self.nav_timer.cancel()
                self.nav_timer = self.create_timer(1.0, self.check_navigation_status)
                
                print(f"开始导航到点位 (ID: {point_id})")
                return True
            else:
                self.get_logger().warn(f"数据库中没有找到ID为 {point_id} 的导航点")
                print(f"找不到ID为 {point_id} 的导航点")
                return False
        
        except Exception as e:
            self.get_logger().error(f"导航到点位 {point_id} 失败: {str(e)}")
            print(f"导航错误: {str(e)}")
            return False
    
    def handle_nav_to_point(self, request, response):
        """处理导航到点位的服务回调"""
        try:
            # 获取最新的已记忆导航点
            cursor = self.conn.cursor()
            cursor.execute('''
                SELECT id, translation_x, translation_y, translation_z, 
                       rotation_x, rotation_y, rotation_z, rotation_w,
                       roll, pitch, yaw
                FROM nav_points 
                WHERE is_remembered = 1
                ORDER BY id DESC 
                LIMIT 1
            ''')
            latest_point = cursor.fetchone()
            
            if latest_point:
                point_id = latest_point[0]
                if self.navigate_to_point_by_id(point_id):
                    response.success = True
                    response.message = f"开始导航到点位 (ID: {point_id})"
                else:
                    response.success = False
                    response.message = f"导航到点位 (ID: {point_id}) 失败"
            else:
                response.success = False
                response.message = "数据库中没有找到已记忆的导航点"
                self.get_logger().warn("没有找到已记忆的导航点")
            
            return response
        
        except Exception as e:
            response.success = False
            response.message = f"导航到点位时发生错误: {str(e)}"
            self.get_logger().error(f"导航到点位失败: {str(e)}")
            return response
    
    def check_navigation_status(self):
        """检查当前导航状态"""
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            self.get_logger().info(f'到达点位 (ID: {self.current_nav_id})，导航结果: {result}')
            print(f"\n导航完成! 点位 (ID: {self.current_nav_id})，结果: {result}")
            print("请输入新的导航点ID (输入'q'退出):")
            
            # 停止定时器
            self.nav_timer.cancel()
            self.nav_timer = None
            self.current_nav_id = None
        else:
            # 获取导航反馈
            feedback = self.navigator.getFeedback()
            remaining = feedback.distance_remaining
            self.get_logger().info(f'导航到点位 (ID: {self.current_nav_id})，剩余距离: {remaining:.2f}米')
    
    def cancel_navigation(self):
        """取消当前导航任务"""
        if self.current_nav_id is not None:
            self.navigator.cancelTask()
            self.get_logger().info(f'取消导航到点位 (ID: {self.current_nav_id})')
            
            # 停止定时器
            if self.nav_timer is not None:
                self.nav_timer.cancel()
                self.nav_timer = None
            
            self.current_nav_id = None
            return True
        return False
    
    def print_available_points(self):
        """打印数据库中所有可用的导航点"""
        try:
            cursor = self.conn.cursor()
            cursor.execute('SELECT id, translation_x, translation_y FROM nav_points')
            points = cursor.fetchall()
            
            if points:
                print("\n可用的导航点:")
                print("ID\tX\t\tY")
                print("-" * 30)
                for point in points:
                    print(f"{point[0]}\t{point[1]:.2f}\t\t{point[2]:.2f}")
            else:
                print("数据库中没有导航点")
        except Exception as e:
            self.get_logger().error(f'获取导航点列表失败: {str(e)}')
            print(f"获取导航点列表错误: {str(e)}")
    
    def __del__(self):
        """析构函数，确保关闭数据库连接"""
        self.running = False
        try:
            if hasattr(self, 'conn'):
                self.conn.close()
                self.get_logger().info('数据库连接已关闭')
        except Exception as e:
            self.get_logger().error(f'关闭数据库连接时出错: {str(e)}')

def main():
    """主函数"""
    rclpy.init()
    node = NavToPointNode()
    
    try:
        # 打印所有可用导航点
        node.print_available_points()
        
        node.get_logger().info('开始运行导航到点位节点...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在关闭节点...')
        node.cancel_navigation()  # 确保取消任何正在进行的导航
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()