#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Nav2InitPointNode(Node):
    def __init__(self):
        super().__init__('nav2_init_point_node')
        
        # 创建回调组以提高服务处理效率
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建服务
        self.srv = self.create_service(
            Trigger, 
            '/init_point', 
            self.handle_init_point,
            callback_group=self.callback_group
        )
        
        # 初始化导航器（延迟初始化）
        self.nav = None
        
        self.get_logger().info("Nav2 初始化节点已启动，等待通过 /init_point 服务触发")

    def handle_init_point(self, request, response):
        """处理初始化位姿请求的回调函数"""
        self.get_logger().info("收到初始化机器人位姿请求")
        
        try:
            # 如果导航器尚未创建，则创建
            if self.nav is None:
                self.nav = BasicNavigator()
                self.get_logger().info("创建BasicNavigator实例")
            
            # 创建初始位姿
            init_pose = PoseStamped()
            init_pose.header.frame_id = "map"
            init_pose.header.stamp = self.get_clock().now().to_msg()
            init_pose.pose.position.x = 0.0
            init_pose.pose.position.y = 0.0
            init_pose.pose.orientation.w = 1.0
            
            # 设置初始位姿
            self.nav.setInitialPose(init_pose)
            self.get_logger().info(f"已设置初始位姿: x={init_pose.pose.position.x}, y={init_pose.pose.position.y}")
            
            # 等待Nav2激活
            self.get_logger().info("等待Nav2系统激活...")
            self.nav.waitUntilNav2Active()
            self.get_logger().info("Nav2系统已激活")
            
            # 设置响应
            response.success = True
            response.message = "成功初始化Nav2导航系统，设置初始位姿为 (0, 0)"

            self.get_logger().info(f"初始化成功: {response.message}")
            self.get_logger().info(f"初始化成功: {response.success}")
            self.get_logger().info("Nav2系统已成功初始化")
            
            return response
        
        except Exception as e:
            # 处理可能的异常
            response.success = False
            response.message = f"初始化失败：{str(e)}"
            self.get_logger().error(f"Nav2初始化失败：{str(e)}")
            return response

def main():
    rclpy.init()
    
    # 创建节点
    node = Nav2InitPointNode()
    
    # 使用多线程执行器以提高性能
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("开始运行服务...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，关闭服务...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()