def navigate_to_point(self, point):
        """导航到指定点位"""
        self.get_logger().info(f"开始导航到点: x={point.point.x}, y={point.point.y}")
        
        # 记录当前位置作为起点
        start_pos = AGV_SDK.get_position()
        self.get_logger().info(f"当前位置: x={start_pos['x']:.3f}, y={start_pos['y']:.3f}, theta={start_pos['theta']:.3f}")
        
        # 计算相对位移
        dx = point.point.x - start_pos['x']
        dy = point.point.y - start_pos['y']
        distance = (dx**2 + dy**2)**0.5
        
        # 计算目标方向角度（弧度）
        target_angle = math.atan2(dy, dx)
        
        # 当前角度与目标角度的差值
        angle_diff = target_angle - start_pos['theta']
        # 标准化到[-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        self.get_logger().info(f"目标距离: {distance:.3f}米, 方向角度: {math.degrees(target_angle):.2f}度")
        self.get_logger().info(f"需要旋转: {math.degrees(angle_diff):.2f}度")
        
        try:
            # 首先旋转到目标方向
            if abs(angle_diff) > 0.1:  # 如果角度差大于0.1弧度（约5.7度）
                self.get_logger().info("旋转到目标方向...")
                # 根据角度差正负确定旋转方向
                rotation_speed = 0.3 if angle_diff > 0 else -0.3
                self.agv_client.publish_velocity(0.0, rotation_speed)
                
                # 计算旋转时间
                rotation_time = abs(angle_diff) / abs(rotation_speed)
                self.get_logger().info(f"旋转时间预计: {rotation_time:.2f}秒")
                
                # 旋转指定时间
                time.sleep(rotation_time)
                AGV_SDK.stop()
                time.sleep(0.5)  # 稳定一下
            
            # 然后前进到目标点
            if distance > 0.1:  # 如果距离大于0.1米
                self.get_logger().info("前进到目标点...")
                linear_speed = 0.2  # 设置一个适中的线速度
                self.agv_client.publish_velocity(linear_speed, 0.0)
                
                # 计算行进时间
                travel_time = distance / linear_speed
                self.get_logger().info(f"行进时间预计: {travel_time:.2f}秒")
                
                # 前进指定时间
                time.sleep(travel_time)
                AGV_SDK.stop()
            
            # 到达目标点
            self.get_logger().info("已到达目标点附近")
            
            # 获取最终位置
            end_pos = AGV_SDK.get_position()
            self.get_logger().info(f"最终位置: x={end_pos['x']:.3f}, y={end_pos['y']:.3f}, theta={end_pos['theta']:.3f}")
            
            # 计算误差
            final_dx = point.point.x - end_pos['x']
            final_dy = point.point.y - end_pos['y']
            final_distance = (final_dx**2 + final_dy**2)**0.5
            
            self.get_logger().info(f"到达误差: {final_distance:.3f}米")
            
            return True
        except Exception as e:
            self.get_logger().error(f"导航过程中发生错误: {e}")
            return False#!/usr/bin/env python3
"""
差分驱动机器人SDK测试脚本
========================
此脚本用于测试AGV_SDK的各项功能，使用MultiThreadedExecutor处理ROS回调
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

import AGV_SDK
import copy

class AGVTestNode(Node):
    """
    ROS2 节点用于测试AGV_SDK功能
    
    此节点:
    - 提供测试各种AGV功能的方法
    - 订阅位置和状态反馈话题
    - 发布运动控制命令
    """
    
    def __init__(self, ros_initialized=False):
        super().__init__('agv_test_node')
        
        # 设置日志级别
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info("初始化AGV测试节点...")
        
        # 初始化AGV_SDK客户端
        try:
            self.agv_client = AGV_SDK.get_client(initialized_ros=ros_initialized)
            self.get_logger().info("AGV客户端初始化成功")
        except Exception as e:
            self.get_logger().error(f"AGV客户端初始化失败: {e}")
            raise
        
        # 初始化TF相关组件
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 状态监控定时器 - 每10秒打印系统状态
        self.status_timer = self.create_timer(10.0, self.report_status)
        
        # 创建订阅
        self.target_point_sub = self.create_subscription(
            PointStamped, 
            '/target_point', 
            self.target_point_callback, 
            1,
            callback_group=self.agv_client.callback_group
        )
        self.get_logger().info("已订阅话题: /target_point")
        
        # 创建位置反馈订阅
        self.position_feedback_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.position_feedback_callback, 
            1,
            callback_group=self.agv_client.callback_group
        )
        self.get_logger().info("已订阅话题: /joint_states")
        
        # 创建命令接收确认订阅
        self.cmd_received_sub = self.create_subscription(
            Bool, 
            '/cmd_received', 
            self.cmd_received_callback, 
            1,
            callback_group=self.agv_client.callback_group
        )
        self.get_logger().info("已订阅话题: /cmd_received")
        
        # 初始化状态变量
        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        self.wheel_status = {'left_velocity': 0.0, 'right_velocity': 0.0}
        self.cmd_received = False
        self.process_running = False
        self.logged_msg_structure = False  # 用于跟踪是否已记录消息结构
        
        # 计数器，用于记录接收的消息数量
        self.message_counters = {
            'joint_states': 0,
            'cmd_received': 0,
            'target_point': 0
        }
        
    def position_feedback_callback(self, msg):
        """处理位置反馈信息"""
        # 安全地处理joint_states消息
        try:
            # 更新消息计数
            self.message_counters['joint_states'] += 1
            
            # 首先检查position数组是否存在且长度足够
            if hasattr(msg, 'position') and len(msg.position) > 0:
                # 根据实际可用的数据更新位置信息
                position_len = len(msg.position)
                
                # 打印一次消息结构以便调试
                if not hasattr(self, 'logged_msg_structure'):
                    self.get_logger().info(f"JointState结构: 名称数量={len(msg.name) if hasattr(msg, 'name') else 0}, "
                                           f"位置数量={position_len}, "
                                           f"速度数量={len(msg.velocity) if hasattr(msg, 'velocity') else 0}")
                    
                    # 如果有名称，打印它们以便调试
                    if hasattr(msg, 'name') and len(msg.name) > 0:
                        self.get_logger().info(f"Joint名称: {', '.join(msg.name)}")
                    
                    self.logged_msg_structure = True
                
                # 安全地获取位置数据
                if position_len > 0:
                    self.current_position['x'] = msg.position[0]
                if position_len > 1:
                    self.current_position['y'] = msg.position[1]
                if position_len > 2:
                    self.current_position['theta'] = msg.position[2]
            
            # 检查velocity数组是否存在且长度足够
            if hasattr(msg, 'velocity') and len(msg.velocity) > 0:
                velocity_len = len(msg.velocity)
                
                # 安全地获取速度数据
                if velocity_len > 0:
                    self.wheel_status['left_velocity'] = msg.velocity[0]
                if velocity_len > 1:
                    self.wheel_status['right_velocity'] = msg.velocity[1]
                
                # 如果有足够的数据，可以计算估计的线速度和角速度
                if velocity_len > 1:
                    # 简单的差分驱动运动学模型
                    wheel_radius = 0.1  # 估计值，需要根据实际情况调整
                    wheel_separation = 0.5  # 估计值，需要根据实际情况调整
                    
                    left_vel = msg.velocity[0]
                    right_vel = msg.velocity[1]
                    
                    # 计算线速度和角速度
                    self.current_velocity['linear'] = wheel_radius * (left_vel + right_vel) / 2.0
                    self.current_velocity['angular'] = wheel_radius * (right_vel - left_vel) / wheel_separation
            
        except Exception as e:
            self.get_logger().error(f"解析位置反馈失败: {e}")
    
    def cmd_received_callback(self, msg):
        """处理命令接收确认"""
        self.cmd_received = msg.data
        self.message_counters['cmd_received'] += 1
        self.get_logger().info(f"命令接收状态: {self.cmd_received}")
    
    def target_point_callback(self, msg):
        """处理目标点消息"""
        self.message_counters['target_point'] += 1
        self.get_logger().info(f"收到目标点: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")
        
        if not self.process_running:
            self.process_running = True
            
            # 记录原始点，用于调试
            self.publish_transform([msg.point.x, msg.point.y, msg.point.z], "map", "original_point")
            
            try:
                # 转换坐标系
                self.get_logger().info(f"尝试将点从 'map' 转换到 'base_link'")
                transformed_point = self.transform_point("map", "base_link", msg.point)
                
                if transformed_point:
                    self.get_logger().info(f"转换后的目标点: x={transformed_point.point.x}, y={transformed_point.point.y}")
                    self.publish_transform(
                        [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z], 
                        "base_link", 
                        "transformed_point"
                    )
                    self.navigate_to_point(transformed_point)
                else:
                    self.get_logger().error("点转换失败，无法导航")
            except Exception as e:
                self.get_logger().error(f"处理目标点时发生错误: {e}")
            finally:
                self.process_running = False
        else:
            self.get_logger().warn("上一个导航过程尚未完成，请稍后再试。")
    
    def publish_transform(self, translation, parent_frame="map", child_frame="target_point"):
        """发布TF变换"""
        transform = TransformStamped()
        transform.header.stamp = self.agv_client.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
    
    def transform_point(self, source_frame, target_frame, point):
        """转换点的坐标系"""
        # 创建PointStamped消息
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = self.agv_client.get_clock().now().to_msg()
        point_stamped.point = copy.deepcopy(point)
        
        # 输出当前可用的TF帧
        buffer_time = rclpy.time.Time()
        try:
            frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().debug(f"可用的TF帧:\n{frames}")
        except Exception as e:
            self.get_logger().warn(f"无法获取TF帧列表: {e}")
        
        # 等待变换可用
        try:
            self.get_logger().info(f"检查是否可以从 '{source_frame}' 转换到 '{target_frame}'")
            if self.tf_buffer.can_transform(
                target_frame, 
                source_frame, 
                buffer_time,
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.get_logger().info(f"'{source_frame}'到'{target_frame}'的变换可用")
            else:
                self.get_logger().error(f"'{source_frame}'到'{target_frame}'的变换不可用，但没有抛出异常")
                return None
        except Exception as e:
            self.get_logger().error(f"变换不可用: {e}")
            # 尝试查找替代帧
            try:
                available_frames = self.tf_buffer._buffer.all_frames_as_yaml()
                self.get_logger().info(f"可用帧: {available_frames}")
            except:
                pass
            return None
        
        # 查找变换
        try:
            self.get_logger().info(f"查找从 '{source_frame}' 到 '{target_frame}' 的变换")
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                buffer_time
            )
            self.get_logger().info(f"成功获取变换: {transform.transform.translation}")
        except Exception as e:
            self.get_logger().error(f"查找变换失败: {e}")
            return None
        
        # 应用变换
        try:
            self.get_logger().info("应用变换到点")
            transformed_point = do_transform_point(point_stamped, transform)
            self.get_logger().info(f"变换成功: 原始点 [{point.x}, {point.y}, {point.z}] -> 转换点 [{transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z}]")
            return transformed_point
        except Exception as e:
            self.get_logger().error(f"应用变换失败: {e}")
            return None
    
    def test_velocity_control(self):
        """测试速度控制功能"""
        self.get_logger().info("\n===== 测试速度控制功能 =====")
        
        # 测试不同速度的前进
        for speed in [0.1, 0.2, 0.3]:
            self.get_logger().info(f"以 {speed} m/s 的速度前进...")
            self.agv_client.publish_velocity(speed, 0.0)
            time.sleep(2)
            
            # 获取当前速度
            velocity = self.agv_client.get_current_velocity()
            self.get_logger().info(f"测量速度: 线速度={velocity['linear']:.3f} m/s, 角速度={velocity['angular']:.3f} rad/s")
            
            AGV_SDK.stop()
            time.sleep(1)
        
        # 测试不同速度的旋转
        for speed in [0.2, 0.4, 0.6]:
            self.get_logger().info(f"以 {speed} rad/s 的速度左转...")
            self.agv_client.publish_velocity(0.0, speed)
            time.sleep(2)
            
            # 获取当前速度
            velocity = self.agv_client.get_current_velocity()
            self.get_logger().info(f"测量速度: 线速度={velocity['linear']:.3f} m/s, 角速度={velocity['angular']:.3f} rad/s")
            
            AGV_SDK.stop()
            time.sleep(1)
        
        self.get_logger().info("速度控制功能测试完成")
    
    def wait_for_cmd_received(self, timeout=5.0):
        """等待命令接收确认"""
        self.cmd_received = False
        start_time = time.time()
        
        while not self.cmd_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.cmd_received:
            self.get_logger().info("命令已被接收")
            return True
        else:
            self.get_logger().warn(f"等待命令接收超时 ({timeout}秒)")
            return False
    
    def test_basic_movement(self):
        """测试基本移动功能"""
        self.get_logger().info("\n===== 测试基本移动功能 =====")
        
        # 测试前进
        self.get_logger().info("1. 前进测试 (3秒)")
        AGV_SDK.move_forward(10.0)  # 使用SDK提供的移动函数
        time.sleep(3)
        AGV_SDK.stop()
        self.get_logger().info("前进测试完成")
        
        # 测试后退
        self.get_logger().info("2. 后退测试 (3秒)")
        AGV_SDK.move_backward(10.0)  # 使用SDK提供的后退函数
        time.sleep(3)
        AGV_SDK.stop()
        self.get_logger().info("后退测试完成")
        
        # 测试左转
        self.get_logger().info("3. 左转测试 (3秒)")
        AGV_SDK.turn_left(0.5)  # 使用SDK提供的左转函数
        time.sleep(3)
        AGV_SDK.stop()
        self.get_logger().info("左转测试完成")
        
        # 测试右转
        self.get_logger().info("4. 右转测试 (3秒)")
        AGV_SDK.turn_right(0.5)  # 使用SDK提供的右转函数
        time.sleep(3)
        AGV_SDK.stop()
        self.get_logger().info("右转测试完成")
        
        # 测试直接发布速度命令
        self.get_logger().info("5. 直接速度控制测试 (3秒)")
        self.agv_client.publish_velocity(0.2, 0.1)  # 前进并轻微左转
        time.sleep(3)
        self.agv_client.publish_velocity(0.0, 0.0)  # 停止
        self.get_logger().info("直接速度控制测试完成")
        
        # 测试车轮速度控制
        self.get_logger().info("6. 车轮速度控制测试 (3秒)")
        self.agv_client.publish_wheel_command(0.3, 0.3)  # 左右轮同速
        time.sleep(3)
        self.agv_client.publish_wheel_command(0.0, 0.0)  # 停止
        self.get_logger().info("车轮速度控制测试完成")
        
        self.get_logger().info("基本移动功能测试完成")
    
    def test_position_feedback(self):
        """测试位置反馈功能"""
        self.get_logger().info("\n===== 测试位置反馈功能 =====")
        
        # 获取初始位置
        initial_pos = AGV_SDK.get_position()
        self.get_logger().info(f"初始位置: x={initial_pos['x']:.3f}, y={initial_pos['y']:.3f}, theta={initial_pos['theta']:.3f}")
        
        # 获取初始车轮状态
        wheel_feedback = self.agv_client.get_wheel_feedback()
        self.get_logger().info(f"初始车轮状态:")
        for key, value in wheel_feedback.items():
            self.get_logger().info(f"  {key}: {value:.3f}")
        
        # 前进一小段距离
        self.get_logger().info("前进中...")
        AGV_SDK.move_forward(20.0)
        time.sleep(3)
        AGV_SDK.stop()
        
        # 获取移动后位置
        new_pos = AGV_SDK.get_position()
        self.get_logger().info(f"移动后位置: x={new_pos['x']:.3f}, y={new_pos['y']:.3f}, theta={new_pos['theta']:.3f}")
        
        # 获取移动后车轮状态
        new_wheel_feedback = self.agv_client.get_wheel_feedback()
        self.get_logger().info(f"移动后车轮状态:")
        for key, value in new_wheel_feedback.items():
            self.get_logger().info(f"  {key}: {value:.3f}")
        
        # 计算位移差
        dx = new_pos['x'] - initial_pos['x']
        dy = new_pos['y'] - initial_pos['y']
        dtheta = new_pos['theta'] - initial_pos['theta']
        
        self.get_logger().info(f"位移差: dx={dx:.3f}, dy={dy:.3f}, dtheta={dtheta:.3f}")
        
        # 获取当前速度
        current_velocity = self.agv_client.get_current_velocity()
        self.get_logger().info(f"当前速度: 线速度={current_velocity['linear']:.3f}, 角速度={current_velocity['angular']:.3f}")
        
        self.get_logger().info("位置反馈功能测试完成")
    
    def test_navigation(self):
        """测试导航功能"""
        self.get_logger().info("\n===== 测试导航功能 =====")
        
        # 初始化位姿
        self.get_logger().info("初始化机器人位姿...")
        success, message = AGV_SDK.initialize_pose()
        self.get_logger().info(f"初始化结果: {'成功' if success else '失败'}, {message}")
        
        # 记住当前位置
        self.get_logger().info("记录当前位置为起点...")
        success, message = AGV_SDK.remember_point()
        self.get_logger().info(f"记录结果: {'成功' if success else '失败'}, {message}")
        
        # 假设第一个点的ID为0
        start_point_id = 0
        
        # 移动到新位置
        self.get_logger().info("移动到新位置...")
        AGV_SDK.move_forward(20.0)
        time.sleep(5)
        AGV_SDK.stop()
        
        # 获取当前位置
        current_pos = AGV_SDK.get_position()
        self.get_logger().info(f"新位置: x={current_pos['x']:.3f}, y={current_pos['y']:.3f}, theta={current_pos['theta']:.3f}")
        
        # 记住当前位置
        self.get_logger().info("记录当前位置为目标点...")
        success, message = AGV_SDK.remember_point()
        self.get_logger().info(f"记录结果: {'成功' if success else '失败'}, {message}")
        
        # 假设第二个点的ID为1
        target_point_id = 1
        
        # 导航回起始点
        self.get_logger().info(f"导航回起始点 (ID: {start_point_id})...")
        success, message = AGV_SDK.navigate_to(start_point_id)
        self.get_logger().info(f"导航结果: {'成功' if success else '失败'}, {message}")
        
        # 导航到目标点
        self.get_logger().info(f"导航到目标点 (ID: {target_point_id})...")
        success, message = AGV_SDK.navigate_to(target_point_id)
        self.get_logger().info(f"导航结果: {'成功' if success else '失败'}, {message}")
        
        self.get_logger().info("导航功能测试完成")
    
    def run_all_tests(self):
        """运行所有测试"""
        try:
            self.get_logger().info("开始测试差分驱动机器人SDK...")
            
            # 执行各项测试
            self.test_basic_movement()
            self.test_position_feedback()
            self.test_velocity_control()
            self.test_wheel_velocity_control()
            
            # 如果ROS环境支持导航功能，则测试导航
            try:
                self.test_navigation()
            except Exception as e:
                self.get_logger().error(f"导航功能测试失败: {e}")
            
            self.get_logger().info("\n所有测试完成!")
            
        except KeyboardInterrupt:
            self.get_logger().info("\n测试被用户中断")
        except Exception as e:
            self.get_logger().error(f"\n测试过程中发生错误: {e}")
        finally:
            # 确保机器人停止
            self.get_logger().info("停止机器人...")
            AGV_SDK.stop()
    
    def shutdown(self):
        """清理关闭节点"""
        self.get_logger().info('关闭AGV测试节点')
        AGV_SDK.shutdown()

    def report_status(self):
        """定期报告系统状态"""
        self.get_logger().info("\n===== 系统状态报告 =====")
        self.get_logger().info(f"消息统计: joint_states={self.message_counters['joint_states']}, "
                              f"cmd_received={self.message_counters['cmd_received']}, "
                              f"target_point={self.message_counters['target_point']}")
        
        self.get_logger().info(f"当前位置: x={self.current_position['x']:.3f}, "
                              f"y={self.current_position['y']:.3f}, "
                              f"theta={self.current_position['theta']:.3f}")
        
        self.get_logger().info(f"当前速度: 线速度={self.current_velocity['linear']:.3f}, "
                              f"角速度={self.current_velocity['angular']:.3f}")
        
        self.get_logger().info(f"车轮状态: 左轮速度={self.wheel_status['left_velocity']:.3f}, "
                              f"右轮速度={self.wheel_status['right_velocity']:.3f}")
        
        # 检查TF连接
        try:
            frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().info(f"可用的TF帧数量: {len(frames.splitlines())}")
        except Exception as e:
            self.get_logger().warn(f"获取TF帧列表失败: {e}")
        
        # 检查AGV客户端状态
        if hasattr(self.agv_client, 'is_active') and callable(self.agv_client.is_active):
            self.get_logger().info(f"AGV客户端状态: {'活跃' if self.agv_client.is_active() else '非活跃'}")
        
        self.get_logger().info("===== 状态报告结束 =====\n")

def main(args=None):
    """主函数"""
    # 初始化ROS
    rclpy.init(args=args)
    
    node = None
    
    try:
        # 创建测试节点，告诉SDK ROS已经初始化
        node = AGVTestNode(ros_initialized=True)
        print("节点已准备就绪!!!")
        
        # 尝试获取执行器
        try:
            executor = AGV_SDK.get_executor()
            print("成功获取SDK执行器")
        except Exception as e:
            print(f"无法获取SDK执行器，创建默认执行器: {e}")
            executor = MultiThreadedExecutor(num_threads=4)
        
        # 添加节点到执行器
        executor.add_node(node)
        
        # 尝试添加AGV客户端到执行器
        try:
            executor.add_node(node.agv_client)
            print("已将AGV客户端添加到执行器")
        except Exception as e:
            print(f"无法将AGV客户端添加到执行器: {e}")
        
        # 打印SDK API信息
        print("\n===== AGV SDK API信息 =====")
        print("可用的SDK函数:")
        for func_name in dir(AGV_SDK):
            if not func_name.startswith('_') and callable(getattr(AGV_SDK, func_name)):
                print(f"  - {func_name}")
        
        # 交互式菜单
        while True:
            print("\n===== 差分驱动机器人SDK测试 =====")
            print("1. 测试基本移动功能")
            print("2. 测试位置反馈功能")
            print("3. 测试速度控制功能")
            print("4. 测试车轮速度控制功能")
            print("5. 测试导航功能")
            print("6. 运行所有测试")
            print("7. 启动常规节点运行(等待话题命令)")
            print("0. 退出程序")
            
            choice = input("\n请选择测试功能 (0-7): ")
            
            if choice == '1':
                node.test_basic_movement()
            elif choice == '2':
                node.test_position_feedback()
            elif choice == '3':
                node.test_velocity_control()
            elif choice == '4':
                node.test_wheel_velocity_control()
            elif choice == '5':
                node.test_navigation()
            elif choice == '6':
                node.run_all_tests()
            elif choice == '7':
                print("启动常规运行，等待命令(按Ctrl+C退出)...")
                try:
                    executor.spin()
                except KeyboardInterrupt:
                    print("用户中断运行")
            elif choice == '0':
                print("退出程序...")
                break
            else:
                print("无效选项，请重新选择")
        
    except KeyboardInterrupt:
        print("收到键盘中断")
    except Exception as e:
        print(f"运行时发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("节点关闭中")
        if node:
            try:
                node.shutdown()
                node.destroy_node()
            except Exception as e:
                print(f"关闭节点时发生错误: {e}")
        
        try:
            rclpy.shutdown()
            print("ROS 2成功关闭")
        except Exception as e:
            print(f"关闭ROS 2时发生错误: {e}")

if __name__ == '__main__':
    main()