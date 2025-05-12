#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import sqlite3
import os
# 修改这里 - 从diff_drive_msgs导入NavigateToPoint
from diff_drive_msgs.action import NavigateToPoint
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
import sys
import queue

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
        
        # 使用 ReentrantCallbackGroup 确保回调可以嵌套调用
        callback_group = ReentrantCallbackGroup()
        
        # 创建 Action 服务器
        self._action_server = ActionServer(
            self,
            NavigateToPoint,
            'nav_to_point',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group)
            
        self.get_logger().info('导航到点位 Action 服务已创建')
        
        # 当前导航的点位ID和目标句柄
        self.current_nav_id = None
        self.current_goal_handle = None
        self.is_navigating = False
        self.start_time = None
        
        # 用于线程间通信的队列
        self.cmd_queue = queue.Queue()
        
        # 创建定时器来处理命令队列
        self.cmd_timer = self.create_timer(0.1, self.process_cmd_queue, callback_group=callback_group)
        
        # 启动键盘输入线程
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        self.get_logger().info('键盘输入线程已启动，请输入导航点ID')
        
        # 打印所有可用导航点
        self.print_available_points()
    
    def keyboard_input_loop(self):
        """处理键盘输入的线程"""
        while self.running:
            try:
                print("\n请输入要导航的点位ID (输入'q'退出, 'c'取消导航, 'p'打印点位): ")
                user_input = input().strip()
                
                if user_input.lower() == 'q':
                    self.get_logger().info('收到退出命令')
                    self.running = False
                    rclpy.shutdown()
                    break
                
                if user_input.lower() == 'c':
                    # 将取消导航命令添加到队列
                    self.cmd_queue.put(('cancel', None))
                    continue
                
                if user_input.lower() == 'p':
                    # 将打印点位命令添加到队列
                    self.cmd_queue.put(('print', None))
                    continue
                
                try:
                    point_id = int(user_input)
                    # 将导航命令添加到队列
                    self.cmd_queue.put(('navigate', point_id))
                except ValueError:
                    print("请输入有效的点位ID（整数）")
            except Exception as e:
                self.get_logger().error(f'键盘输入处理错误: {str(e)}')
    
    def process_cmd_queue(self):
        """处理命令队列的回调函数，在主线程中执行"""
        try:
            while not self.cmd_queue.empty():
                cmd, data = self.cmd_queue.get_nowait()
                
                if cmd == 'navigate':
                    self.navigate_to_point_by_id_cmd(data)
                elif cmd == 'cancel':
                    if self.cancel_navigation():
                        print("导航已取消")
                    else:
                        print("没有正在进行的导航")
                elif cmd == 'print':
                    self.print_available_points()
        except Exception as e:
            self.get_logger().error(f'处理命令队列错误: {str(e)}')
    
    def navigate_to_point_by_id_cmd(self, point_id):
        """从命令行输入触发的导航命令处理"""
        # 如果当前有导航进行中，先取消
        if self.is_navigating:
            self.cancel_navigation()
        
        # 通过内部方法执行导航
        success, message = self.navigate_to_point_by_id(point_id)
        if success:
            print(f"开始导航到点位 (ID: {point_id})")
        else:
            print(f"导航错误: {message}")
    
    def navigate_to_point_by_id(self, point_id):
        """根据点位ID导航到指定位置，返回（成功状态，消息）元组"""
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
                
                # 记录当前导航的点位ID和开始时间
                self.current_nav_id = point_id
                self.is_navigating = True
                self.start_time = time.time()
                
                return True, f"开始导航到点位 (ID: {point_id})"
            else:
                self.get_logger().warn(f"数据库中没有找到ID为 {point_id} 的导航点")
                return False, f"找不到ID为 {point_id} 的导航点"
        
        except Exception as e:
            self.get_logger().error(f"导航到点位 {point_id} 失败: {str(e)}")
            return False, f"导航错误: {str(e)}"
    
    def goal_callback(self, goal_request):
        """处理新的导航目标请求"""
        # 获取请求的点位ID
        point_id = goal_request.point_id
        self.get_logger().info(f'收到导航目标请求，点位ID: {point_id}')
        
        # 如果当前有导航正在进行，拒绝新目标
        if self.is_navigating:
            self.get_logger().warn('已有导航任务正在执行，拒绝新目标')
            return GoalResponse.REJECT
        
        # 检查点位是否存在
        cursor = self.conn.cursor()
        cursor.execute('SELECT COUNT(*) FROM nav_points WHERE id = ?', (point_id,))
        count = cursor.fetchone()[0]
        
        if count == 0:
            self.get_logger().warn(f'数据库中没有ID为 {point_id} 的导航点，拒绝目标')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """处理取消导航目标请求"""
        self.get_logger().info('收到取消导航请求')
        
        if self.cancel_navigation():
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT
    
    def execute_callback(self, goal_handle):
        """执行导航目标的回调函数"""
        # 将当前目标句柄保存为类变量
        self.current_goal_handle = goal_handle
        
        # 获取目标点位ID
        point_id = goal_handle.request.point_id
        self.get_logger().info(f'执行导航目标，点位ID: {point_id}')
        
        # 创建结果对象
        result = NavigateToPoint.Result()
        
        # 开始导航
        success, message = self.navigate_to_point_by_id(point_id)
        
        if not success:
            # 如果导航初始化失败，立即返回失败结果
            result.success = False
            result.message = message
            self.get_logger().error(f'导航初始化失败: {message}')
            return result
        
        # 进入导航状态监控循环
        feedback_msg = NavigateToPoint.Feedback()
        
        try:
            while rclpy.ok() and self.is_navigating:
                # 检查是否取消
                if goal_handle.is_cancel_requested:
                    self.cancel_navigation()
                    result.success = False
                    result.message = "导航任务被取消"
                    goal_handle.canceled()
                    self.get_logger().info('导航任务被取消')
                    return result
                
                # 检查导航状态
                if self.navigator.isTaskComplete():
                    # 导航完成
                    self.is_navigating = False
                    nav_result = self.navigator.getResult()
                    
                    if nav_result == "succeeded":
                        result.success = True
                        result.message = f"成功到达点位 (ID: {point_id})"
                        self.get_logger().info(f'导航成功: {result.message}')
                    else:
                        result.success = False
                        result.message = f"导航到点位 (ID: {point_id}) 失败: {nav_result}"
                        self.get_logger().warn(f'导航失败: {result.message}')
                    
                    self.current_nav_id = None
                    goal_handle.succeed()
                    return result
                
                # 获取导航反馈
                nav_feedback = self.navigator.getFeedback()
                if nav_feedback:
                    # 更新反馈消息
                    feedback_msg.distance_remaining = nav_feedback.distance_remaining
                    feedback_msg.navigation_time = time.time() - self.start_time
                    feedback_msg.current_pose = self.navigator.getCurrentPose()
                    
                    # 发送反馈
                    goal_handle.publish_feedback(feedback_msg)
                    
                    # 打印反馈信息
                    self.get_logger().info(
                        f'导航反馈 - 点位ID: {point_id}, 剩余距离: {feedback_msg.distance_remaining:.2f}米, '
                        f'已用时间: {feedback_msg.navigation_time:.1f}秒'
                    )
                
                # 等待一段时间后继续检查
                time.sleep(0.5)
        
        except Exception as e:
            # 发生异常时，确保导航被取消并返回失败结果
            self.get_logger().error(f'导航执行过程中发生错误: {str(e)}')
            self.cancel_navigation()
            result.success = False
            result.message = f"导航执行错误: {str(e)}"
            return result
    
    def cancel_navigation(self):
        """取消当前导航任务"""
        if self.is_navigating:
            self.navigator.cancelTask()
            self.get_logger().info(f'取消导航到点位 (ID: {self.current_nav_id})')
            
            self.is_navigating = False
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