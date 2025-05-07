#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from diff_drive_msgs.msg import WheelCommand  # 导入正确的消息类型
import time

class WheelSpeedController(Node):
    def __init__(self):
        super().__init__('wheel_speed_controller')
        
        # 声明更多必要参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.18),     
                ('wheel_separation', 0.5),
                ('max_linear_speed', 1000.0),  # 最大线速度
                ('max_angular_speed', 1000.0)  # 最大角速度
            ]
        )
        
        # 获取参数
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # 用于存储线速度和角速度
        self.wheel_speed_linear = 0.0
        self.wheel_speed_angular = 0.0
        
        # 用于安全超时
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # 0.5秒超时
        
        # 创建发布者 - 发布轮子速度命令
        self.wheel_pub = self.create_publisher(
            WheelCommand,  # 使用WheelCommand消息类型
            'wheel_cmd',   # 发布到wheel_cmd话题
            10
        )
        
        # 订阅cmd_vel话题 - 注意添加了前导斜杠
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # 修改为正确的Nav2发布话题
            self.cmd_vel_callback,
            10
        )
        
        # 安全计时器 - 检查是否需要停止
        self.timer = self.create_timer(0.1, self.safety_timer_callback)
        
        self.get_logger().info("轮子速度控制器已启动，订阅/cmd_vel话题")
    
    def cmd_vel_callback(self, msg):
        """处理velocity命令并转换为轮子速度"""
        # 更新命令时间戳
        self.last_cmd_time = time.time()
        
        # 提取线速度和角速度并限制范围
        linear_x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        self.wheel_speed_linear = linear_x
        self.wheel_speed_angular = angular_z
        
        # 创建轮子命令消息
        wheels_msg = WheelCommand()
        
        # 修正的差速驱动公式
        # v_left = (v - ω * L/2) / r
        # v_right = (v + ω * L/2) / r
        left_wheel = -(linear_x - angular_z * self.wheel_separation/2) / self.wheel_radius 
        right_wheel = (linear_x + angular_z * self.wheel_separation/2) / self.wheel_radius 
        
        wheels_msg.left_wheel_velocity = float(left_wheel * 50)
        wheels_msg.right_wheel_velocity = float(right_wheel * 50)
        
        # 发布消息
        self.wheel_pub.publish(wheels_msg)
        self.get_logger().debug(f"发布轮子速度: 左={wheels_msg.left_wheel_velocity:.2f}, 右={wheels_msg.right_wheel_velocity:.2f}")

    def safety_timer_callback(self):
        """安全检查 - 如果长时间未收到命令则停止轮子"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            wheels_msg = WheelCommand()
            wheels_msg.left_wheel_velocity = 0.0
            wheels_msg.right_wheel_velocity = 0.0
            self.wheel_pub.publish(wheels_msg)
    
    def stop_wheels(self):
        """停止所有轮子"""
        wheels_msg = WheelCommand()
        wheels_msg.left_wheel_velocity = 0.0
        wheels_msg.right_wheel_velocity = 0.0
        self.wheel_pub.publish(wheels_msg)
        self.get_logger().info("轮子已停止")

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 在退出前停止轮子
        node.stop_wheels()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()