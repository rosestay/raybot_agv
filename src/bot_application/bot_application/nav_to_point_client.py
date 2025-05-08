#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# 修改这里 - 从diff_drive_msgs导入NavigateToPoint
from diff_drive_msgs.action import NavigateToPoint
import sys

class NavToPointActionClient(Node):
    def __init__(self):
        super().__init__('nav_to_point_client')
        
        # 创建 Action 客户端
        self._action_client = ActionClient(self, NavigateToPoint, 'nav_to_point')
        
        self.get_logger().info('等待 Action 服务器启动...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action 服务器已连接!')

    def send_goal(self, point_id):
        """发送导航目标"""
        goal_msg = NavigateToPoint.Goal()
        goal_msg.point_id = point_id
        
        self.get_logger().info(f'发送导航请求，目标点位ID: {point_id}')
        
        # 发送目标并设置回调函数
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        
        # 设置目标响应回调
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """处理目标请求的响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝!')
            return
            
        self.get_logger().info('目标被接受，等待结果...')
        
        # 请求目标执行结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """处理导航结果"""
        result = future.result().result
        status = "成功" if result.success else "失败"
        
        self.get_logger().info(f'导航结果: {status}')
        self.get_logger().info(f'消息: {result.message}')
        
        # 关闭节点
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'导航反馈 - 剩余距离: {feedback.distance_remaining:.2f}米, 已用时间: {feedback.navigation_time:.1f}秒')

def main(args=None):
    rclpy.init(args=args)
    
    # 获取命令行参数
    if len(sys.argv) != 2:
        print("使用方法: ros2 run <包名> nav_to_point_client.py <点位ID>")
        rclpy.shutdown()
        return
    
    try:
        point_id = int(sys.argv[1])
    except ValueError:
        print("错误: 点位ID必须是整数")
        rclpy.shutdown()
        return
    
    # 创建客户端并发送目标
    client = NavToPointActionClient()
    client.send_goal(point_id)
    
    # 开始处理
    rclpy.spin(client)

if __name__ == '__main__':
    main()