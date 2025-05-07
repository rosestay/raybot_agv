#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class Nav2InitPointNode(Node):
    def __init__(self):
        super().__init__('nav2_init_point_node')
        
        # 创建服务
        self.srv = self.create_service(Trigger, '/init_point', self.handle_init_point)
        
        # 初始化导航器（延迟初始化）
        self.nav = None
        
        self.get_logger().info("Nav2 初始化节点已启动，等待通过 /init_point 服务触发")

    def handle_init_point(self, request, response):
        try:
            # 如果导航器尚未创建，则创建
            if self.nav is None:
                self.nav = BasicNavigator()
            
            # 创建初始位姿
            init_pose = PoseStamped()
            init_pose.header.frame_id = "map"
            init_pose.header.stamp = self.get_clock().now().to_msg()
            init_pose.pose.position.x = 0.0
            init_pose.pose.position.y = 0.0
            init_pose.pose.orientation.w = 1.0
            
            # 设置初始位姿
            self.nav.setInitialPose(init_pose)
            
            # 等待Nav2激活
            self.nav.waitUntilNav2Active()
            
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
    node = Nav2InitPointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()