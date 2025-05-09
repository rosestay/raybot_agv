#!/usr/bin/env python3
"""
差分驱动机器人SDK测试脚本
========================
此脚本用于测试AGV_SDK的各项功能，使用MultiThreadedExecutor处理ROS回调
"""

import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from AGV_SDK import init, shutdown, move_forward, move_backward, turn_left, turn_right, stop
from AGV_SDK import get_position, navigate_to, remember_point, initialize_pose
from AGV_SDK import get_client

class SDKTestNode(Node):
    """SDK测试节点类"""
    
    def __init__(self):
        super().__init__('sdk_test_node')
        self.get_logger().info('SDK测试节点已初始化')
    
    def test_basic_movement(self):
        """测试基本移动功能"""
        self.get_logger().info("\n===== 测试基本移动功能 =====")
        
        self.get_logger().info("1. 前进测试 (2秒)")
        move_forward(10.0)  # 低速前进
        time.sleep(3)
        stop()
        self.get_logger().info("前进测试完成")
        
        self.get_logger().info("2. 后退测试 (2秒)")
        move_backward(10.0)  # 低速后退
        time.sleep(3)
        stop()
        self.get_logger().info("后退测试完成")
        
        self.get_logger().info("3. 左转测试 (2秒)")
        turn_left(10.0)  # 低速左转
        time.sleep(3)
        stop()
        self.get_logger().info("左转测试完成")
        
        self.get_logger().info("4. 右转测试 (2秒)")
        turn_right(10.0)  # 低速右转
        time.sleep(3)
        stop()
        self.get_logger().info("右转测试完成")
        
        self.get_logger().info("基本移动功能测试完成")

    def test_position_feedback(self):
        """测试位置反馈功能"""
        self.get_logger().info("\n===== 测试位置反馈功能 =====")
        
        # 获取初始位置
        initial_pos = get_position()
        self.get_logger().info(f"初始位置: x={initial_pos['x']:.3f}, y={initial_pos['y']:.3f}, theta={initial_pos['theta']:.3f}")
        
        # 前进一小段距离
        self.get_logger().info("前进中...")
        move_forward(20.0)
        time.sleep(3)
        stop()
        
        # 获取移动后位置
        new_pos = get_position()
        self.get_logger().info(f"移动后位置: x={new_pos['x']:.3f}, y={new_pos['y']:.3f}, theta={new_pos['theta']:.3f}")
        
        # 计算位移差
        dx = new_pos['x'] - initial_pos['x']
        dy = new_pos['y'] - initial_pos['y']
        dtheta = new_pos['theta'] - initial_pos['theta']
        
        self.get_logger().info(f"位移差: dx={dx:.3f}, dy={dy:.3f}, dtheta={dtheta:.3f}")
        self.get_logger().info("位置反馈功能测试完成")

    def test_wheel_feedback(self):
        """测试车轮反馈信息"""
        self.get_logger().info("\n===== 测试车轮反馈信息 =====")
        
        client = get_client()
        
        # 获取初始车轮状态
        initial_wheel = client.get_wheel_feedback()
        self.get_logger().info("初始车轮状态:")
        for key, value in initial_wheel.items():
            self.get_logger().info(f"  {key}: {value:.3f}")
        
        # 前进一小段距离
        self.get_logger().info("前进中...")
        move_forward(0.2)
        time.sleep(2)
        stop()
        
        # 获取移动后车轮状态
        new_wheel = client.get_wheel_feedback()
        self.get_logger().info("移动后车轮状态:")
        for key, value in new_wheel.items():
            self.get_logger().info(f"  {key}: {value:.3f}")
        
        self.get_logger().info("车轮反馈信息测试完成")

    def test_point_navigation(self):
        """测试点导航功能"""
        self.get_logger().info("\n===== 测试点导航功能 =====")
        
        # 初始化位姿
        self.get_logger().info("初始化机器人位姿...")
        success, message = initialize_pose()
        self.get_logger().info(f"初始化结果: {'成功' if success else '失败'}, {message}")
        
        # 记住当前点位
        self.get_logger().info("记录当前位置为起始点...")
        success, message = remember_point()
        self.get_logger().info(f"记录结果: {'成功' if success else '失败'}, {message}")
        start_point_id = 0  # 假设第一个点的ID为0
        
        # 移动到新位置
        self.get_logger().info("移动到新位置...")
        move_forward(0.2)
        time.sleep(5)
        stop()
        
        # 记住当前点位
        self.get_logger().info("记录当前位置为目标点...")
        success, message = remember_point()
        self.get_logger().info(f"记录结果: {'成功' if success else '失败'}, {message}")
        target_point_id = 1  # 假设第二个点的ID为1
        
        # 导航回起始点
        self.get_logger().info(f"导航回起始点 (ID: {start_point_id})...")
        success, message = navigate_to(start_point_id)
        self.get_logger().info(f"导航结果: {'成功' if success else '失败'}, {message}")
        
        # 导航到目标点
        self.get_logger().info(f"导航到目标点 (ID: {target_point_id})...")
        success, message = navigate_to(target_point_id)
        self.get_logger().info(f"导航结果: {'成功' if success else '失败'}, {message}")
        
        self.get_logger().info("点导航功能测试完成")

    def test_velocity_control(self):
        """测试速度控制功能"""
        self.get_logger().info("\n===== 测试速度控制功能 =====")
        
        client = get_client()
        
        # 测试不同速度的前进
        for speed in [0.1, 0.2, 0.3]:
            self.get_logger().info(f"以 {speed} m/s 的速度前进...")
            client.publish_velocity(speed, 0.0)
            time.sleep(2)
            
            # 获取当前速度
            velocity = client.get_current_velocity()
            self.get_logger().info(f"测量速度: 线速度={velocity['linear']:.3f} m/s, 角速度={velocity['angular']:.3f} rad/s")
            
            stop()
            time.sleep(1)
        
        # 测试不同速度的旋转
        for speed in [0.2, 0.4, 0.6]:
            self.get_logger().info(f"以 {speed} rad/s 的速度左转...")
            client.publish_velocity(0.0, speed)
            time.sleep(2)
            
            # 获取当前速度
            velocity = client.get_current_velocity()
            self.get_logger().info(f"测量速度: 线速度={velocity['linear']:.3f} m/s, 角速度={velocity['angular']:.3f} rad/s")
            
            stop()
            time.sleep(1)
        
        self.get_logger().info("速度控制功能测试完成")

    def test_wheel_velocity_control(self):
        """测试车轮速度控制功能"""
        self.get_logger().info("\n===== 测试车轮速度控制功能 =====")
        
        client = get_client()
        
        # 测试左右轮相同速度
        self.get_logger().info("左右轮相同速度 (直线行驶)...")
        client.publish_wheel_command(0.2, 0.2)
        time.sleep(3)
        
        # 获取车轮反馈
        wheel_feedback = client.get_wheel_feedback()
        self.get_logger().info(f"左轮速度: {wheel_feedback['left_velocity']:.3f}, 右轮速度: {wheel_feedback['right_velocity']:.3f}")
        
        stop()
        time.sleep(1)
        
        # 测试左右轮不同速度
        self.get_logger().info("左右轮不同速度 (转弯)...")
        client.publish_wheel_command(0.3, 0.1)
        time.sleep(3)
        
        # 获取车轮反馈
        wheel_feedback = client.get_wheel_feedback()
        self.get_logger().info(f"左轮速度: {wheel_feedback['left_velocity']:.3f}, 右轮速度: {wheel_feedback['right_velocity']:.3f}")
        
        stop()
        time.sleep(1)
        
        self.get_logger().info("车轮速度控制功能测试完成")

    def run_all_tests(self):
        """运行所有测试"""
        try:
            self.get_logger().info("开始测试差分驱动机器人SDK...")
            
            # 执行各项测试
            self.test_basic_movement()
            self.test_position_feedback()
            self.test_wheel_feedback()
            self.test_velocity_control()
            self.test_wheel_velocity_control()
            
            # 如果ROS环境支持导航功能，则测试导航
            try:
                self.test_point_navigation()
            except Exception as e:
                self.get_logger().error(f"导航功能测试失败，可能是ROS环境不支持: {e}")
            
            self.get_logger().info("\n所有测试完成!")
            
        except KeyboardInterrupt:
            self.get_logger().info("\n测试被用户中断")
        except Exception as e:
            self.get_logger().error(f"\n测试过程中发生错误: {e}")
        finally:
            # 确保机器人停止
            self.get_logger().info("停止机器人...")
            stop()


def main(args=None):
    """主函数"""
    try:
        # 初始化SDK (假设SDK内部已经初始化了ROS 2)
        print("初始化SDK...")
        init()
        time.sleep(1)  # 等待初始化完成
        
        # 创建测试节点
        node = SDKTestNode()
        
        # 创建多线程执行器
        executor = MultiThreadedExecutor(num_threads=4)  # 使用4个线程
        executor.add_node(node)
        
        # 运行测试
        node.run_all_tests()
        
        # 使用多线程执行器运行节点，处理回调
        executor.spin_once()  # 执行一次，处理任何待处理的回调
        
        # 清理节点
        node.destroy_node()
        
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 关闭SDK
        print("关闭SDK...")
        shutdown()
        
        # 关闭ROS 2
        rclpy.shutdown()


if __name__ == "__main__":
    main()