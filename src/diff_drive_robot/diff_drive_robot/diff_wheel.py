#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from diff_drive_msgs.msg import Feedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Vector3, Twist
import tf2_ros
import numpy as np

class WheelOdometryWithTF(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Parameters
        self.declare_parameter('wheel_diameter', 0.18)  # wheel diameter in meters
        self.declare_parameter('wheel_base', 0.5)  # distance between wheels in meters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rotation_correction', 1.5)  # 新增: 旋转修正因子参数
        
        # Get parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.rotation_correction = self.get_parameter('rotation_correction').value  # 获取旋转修正因子
        
        self.wheel_radius = self.wheel_diameter / 2.0
        
        # Odometry state
        self.x = 0.0  # x position (forward)
        self.y = 0.0  # y position (left)
        self.theta = 0.0  # orientation (counter-clockwise from x-axis)
        
        # Previous wheel positions and timestamp
        self.prev_left_position = None
        self.prev_right_position = None
        self.prev_timestamp = None
        
        # TF2 Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Odometry publisher
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscription to wheel feedback
        self.subscription = self.create_subscription(
            Feedback,
            '/wheel_feedback',
            self.odometry_callback,
            10
        )
        
        self.get_logger().info(f'Wheel Odometry Node Initialized')
        self.get_logger().info(f'Odom Frame: {self.odom_frame}, Base Frame: {self.base_frame}')
        self.get_logger().info(f'Rotation Correction Factor: {self.rotation_correction}')
    
    def normalize_angle(self, angle):
        """Normalize angle to [0, 2π) range."""
        return angle % (2 * math.pi)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion (ROS standard)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]
    
    def publish_tf_transform(self, timestamp):
        # Create transform stamped message
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        # Set translation (ensure correct coordinate system)
        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0
        
        # Convert orientation to quaternion (positive counter-clockwise)
        quaternion = self.euler_to_quaternion(0, 0, self.theta)
        
        # Set rotation
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
    
    def odometry_callback(self, msg):
        # Get current timestamp
        current_timestamp = self.get_clock().now().to_msg()
        current_time = self.get_clock().now()
        
        left_position = msg.left_wheel_position
        right_position = msg.right_wheel_position
        
        # Skip first iteration
        if (self.prev_left_position is None) or (self.prev_right_position is None):
            self.prev_left_position = left_position
            self.prev_right_position = right_position
            self.prev_timestamp = current_time
            return
        
        # 计算时间差
        dt = (current_time - self.prev_timestamp).nanoseconds / 1e9  # 转换为秒
        if dt <= 0:
            return
        
        # Calculate angular displacements
        # Correct for wheel rotation direction
        delta_left = self.prev_left_position - left_position  # Reversed for forward motion
        delta_right = right_position - self.prev_right_position

        # Calculate linear displacements - 使用标准数学库常量
        delta_s_left = delta_left * self.wheel_radius * math.pi / 180.0  # 使用math.pi并保证精度
        delta_s_right = delta_right * self.wheel_radius * math.pi / 180.0  # 统一精度
        
        # Calculate average linear displacement and angular displacement
        # Positive rotation is counter-clockwise
        delta_s = (delta_s_left + delta_s_right) / 2.0
        
        # 修改: 使用1.5的修正因子 - 这是根据反馈调整的值
        delta_theta = (delta_s_right - delta_s_left) / self.wheel_base * self.rotation_correction
        
        # 计算实际的轮距 - 用于调试
        if abs(delta_s_right - delta_s_left) > 0.001 and abs(delta_theta) > 0.001:
            actual_wheel_base = abs(delta_s_right - delta_s_left) / (abs(delta_theta) / self.rotation_correction)
            self.get_logger().debug(f'Measured wheel_base: {actual_wheel_base:.4f}m vs configured: {self.wheel_base:.4f}m')
        
        # 计算线速度和角速度
        v_x = delta_s / dt  # 前向线速度
        v_theta = delta_theta / dt  # 角速度
        
        # Small angle approximation for pose update
        if abs(delta_theta) < 1e-6:
            # Straight line motion
            delta_x = delta_s * math.cos(self.theta)
            delta_y = delta_s * math.sin(self.theta)
        else:
            # Circular arc motion
            R = delta_s / delta_theta  # Instantaneous rotation radius
            delta_x = R * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = R * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        
        # Update pose
        self.x += delta_x
        self.y += delta_y
        # Ensure counter-clockwise rotation
        self.theta = self.normalize_angle(self.theta + delta_theta)
        
        # Prepare odometry message
        odom = Odometry()
        odom.header.stamp = current_timestamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set position
        odom.pose.pose.position = Point(x=float(self.x), y=float(self.y), z=0.0)
        
        # Convert orientation to quaternion
        quaternion = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], 
            z=quaternion[2], w=quaternion[3]
        )
        
        # 添加速度信息 - EKF融合需要
        # 在机器人坐标系中的线速度和角速度
        odom.twist.twist = Twist(
            linear=Vector3(x=float(v_x), y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(v_theta))
        )
        
        # Publish odometry
        self.odometry_publisher.publish(odom)
        
        # Publish TF transform
        self.publish_tf_transform(current_timestamp)
        
        # Update previous wheel positions and timestamp
        self.prev_left_position = left_position
        self.prev_right_position = right_position
        self.prev_timestamp = current_time
        
        # Log current pose and wheel angles
        self.get_logger().info(
            f'Odometry - x: {self.x:.3f}, y: {self.y:.3f}, theta: {math.degrees(self.theta):.3f}°, '
            f'v_x: {v_x:.3f}, v_theta: {math.degrees(v_theta):.3f}°/s, '
            f'L: {left_position:.1f}, R: {right_position:.1f}, dL: {delta_left:.1f}, dR: {delta_right:.1f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    wheel_odometry = WheelOdometryWithTF()
    
    try:
        rclpy.spin(wheel_odometry)
    except KeyboardInterrupt:
        wheel_odometry.get_logger().info('Wheel Odometry Node Interrupted')
    finally:
        wheel_odometry.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()