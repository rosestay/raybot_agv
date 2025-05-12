#!/usr/bin/env python3
"""
Differential Drive Robot SDK
===========================
"""

import rclpy
import time
import math
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import standard ROS2 message types
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

# Import custom message types
from diff_drive_msgs.msg import WheelCommand, Feedback
from diff_drive_msgs.action import NavigateToPoint


class DiffDriveRobotClient(Node):
    """Differential Drive Robot Client"""
    
    def __init__(self):
        super().__init__('diff_drive_robot_client')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheel_cmd_pub = self.create_publisher(WheelCommand, '/wheel_cmd', 10)
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.wheel_feedback_sub = self.create_subscription(
            Feedback, '/wheel_feedback', self._wheel_feedback_callback, 10)
        
        # Create Action client
        self.navigate_client = ActionClient(
            self, NavigateToPoint, 'nav_to_point', callback_group=self.callback_group)
        
        # Create service clients
        self.init_pose_client = self.create_client(
            Trigger, '/init_point', callback_group=self.callback_group)
        self.rem_point_client = self.create_client(
            Trigger, '/rem_point', callback_group=self.callback_group)
        
        # State variables
        self._pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._velocity = {'linear': 0.0, 'angular': 0.0}
        self._wheel_feedback = {
            'left_position': 0.0,
            'left_velocity': 0.0,
            'right_position': 0.0,
            'right_velocity': 0.0,
        }
        self._lock = threading.Lock()
        
        # Navigation state
        self.execution_complete = False
        self.execution_success = False
        
        self.get_logger().info("Differential Drive Robot Client initialized")
    
    def _odom_callback(self, msg):
        """Odometry message callback handler"""
        with self._lock:
            # Extract position
            self._pose['x'] = msg.pose.pose.position.x
            self._pose['y'] = msg.pose.pose.position.y
            
            # Extract orientation (quaternion to yaw)
            q = msg.pose.pose.orientation
            self._pose['theta'] = self._quaternion_to_yaw(q)
            
            # Extract velocity
            self._velocity['linear'] = msg.twist.twist.linear.x
            self._velocity['angular'] = msg.twist.twist.angular.z
    
    def _wheel_feedback_callback(self, msg):
        """Wheel feedback message callback handler"""
        with self._lock:
            self._wheel_feedback['left_position'] = msg.left_wheel_position
            self._wheel_feedback['left_velocity'] = msg.left_wheel_velocity
            self._wheel_feedback['right_position'] = msg.right_wheel_position
            self._wheel_feedback['right_velocity'] = msg.right_wheel_velocity
    
    def _quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle (radians)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)
    
    def publish_wheel_command(self, left_velocity, right_velocity):
        """Publish wheel velocity command"""
        msg = WheelCommand()
        msg.left_wheel_velocity = float(left_velocity)
        msg.right_wheel_velocity = float(right_velocity)
        self.wheel_cmd_pub.publish(msg)
    
    def get_current_pose(self):
        """Get current position"""
        with self._lock:
            return self._pose.copy()
    
    def get_current_velocity(self):
        """Get current velocity"""
        with self._lock:
            return self._velocity.copy()
    
    def get_wheel_feedback(self):
        """Get wheel feedback information"""
        with self._lock:
            return self._wheel_feedback.copy()
    
    def init_robot_pose(self):
        """Initialize robot pose"""
        request = Trigger.Request()
        
        # Wait for service to be available
        self.init_pose_client.wait_for_service()
        
        # Send request
        future = self.init_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        return response.success, response.message
    
    def remember_current_point(self):
        """Remember current position as a navigation point"""
        request = Trigger.Request()
        
        # Wait for service to be available
        self.rem_point_client.wait_for_service()
        
        # Send request
        future = self.rem_point_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        return response.success, response.message
    
    def navigate_to_point(self, point_id, wait=True):
        """Navigate to specified point"""
        # Wait for Action server to be available
        self.navigate_client.wait_for_server()
        
        # Create goal message
        goal_msg = NavigateToPoint.Goal()
        goal_msg.point_id = point_id
        
        # Reset execution state
        self.execution_complete = False
        self.execution_success = False
        
        # Send goal and set callbacks
        send_goal_future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        # If waiting for completion is required
        if wait:
            # Wait for navigation to complete
            timeout = 60.0  # Maximum wait time of 60 seconds
            start_time = time.time()
            
            while not self.execution_complete and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.execution_complete:
                if self.execution_success:
                    return True, "Navigation completed successfully"
                else:
                    return False, "Navigation failed"
            else:
                return False, "Navigation timeout"
            
        return True, "Navigation request sent"
    
    def _goal_response_callback(self, future):
        """Goal response callback"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.execution_complete = True
            self.execution_success = False
            self.get_logger().error('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Request result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        """Get result callback"""
        result = future.result().result
        
        self.execution_complete = True
        self.execution_success = result.success
        
        if result.success:
            self.get_logger().info(f'Navigation success: {result.message}')
        else:
            self.get_logger().error(f'Navigation failed: {result.message}')
    
    def _feedback_callback(self, feedback_msg):
        """Feedback callback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigation feedback: Remaining distance={feedback.distance_remaining:.2f}m, '
            f'Time elapsed={feedback.navigation_time:.1f} seconds'
        )
    
    def cancel_navigation(self):
        """Cancel current navigation task"""
        # TODO: Implement navigation cancellation
        self.get_logger().info('Canceling navigation')
        return True


# Global client instance
_client = None
_executor = None
_initialized = False

def get_executor():
    global _executor

    return _executor


def init(initialized_ros=False):
    """Initialize SDK
    
    Args:
        initialized_ros: Set to True if ROS is already initialized externally
    """
    global _client, _initialized, _executor
    
    if not _initialized:
        # Only initialize ROS if not already initialized externally
        if not initialized_ros:
            rclpy.init()
        
        _client = DiffDriveRobotClient()
        _executor = MultiThreadedExecutor()
       
        print("Differential Drive Robot SDK initialized")
        _initialized = True
    
    return _client

def get_client(initialized_ros=True):
    """Get client instance
    
    Args:
        initialized_ros: Set to True if ROS is already initialized externally
    """
    global _client, _initialized
    
    if not _initialized:
        init(initialized_ros=initialized_ros)
    
    return _client

def shutdown():
    """Shutdown SDK"""
    global _client, _initialized, _executor
    
    if _initialized and _client is not None:
        _executor.shutdown()
        _client.destroy_node()
        rclpy.shutdown()
        _initialized = False
        print("Differential Drive Robot SDK shutdown")


# Simple API wrapper functions
def move_forward(speed=0.5):
    """Move forward"""
    client = get_client()
    client.publish_velocity(speed, 0.0)

def move_backward(speed=0.5):
    """Move backward"""
    client = get_client()
    client.publish_velocity(-speed, 0.0)

def turn_left(speed=0.5):
    """Turn left"""
    client = get_client()
    client.publish_velocity(0.0, speed)

def turn_right(speed=0.5):
    """Turn right"""
    client = get_client()
    client.publish_velocity(0.0, -speed)

def stop():
    """Stop"""
    client = get_client()
    client.publish_velocity(0.0, 0.0)

def get_position():
    """Get current position"""
    client = get_client()
    return client.get_current_pose()

def navigate_to(point_id, wait=True):
    """Navigate to specified point"""
    client = get_client()
    return client.navigate_to_point(point_id, wait)

def remember_point():
    """Remember current position"""
    client = get_client()
    return client.remember_current_point()

def initialize_pose():
    """Initialize robot pose"""
    client = get_client()
    return client.init_robot_pose()


# Example usage
# if __name__ == '__main__':
#     try:
#         # Initialize SDK
#         init()
        
#         # Use SDK functionality
#         print("Current position:", get_position())
        
#         print("Moving forward...")
#         move_forward(0.3)
#         time.sleep(2.0)
        
#         print("Stopping...")
#         stop()
        
#         print("Remembering current position...")
#         success, message = remember_point()
#         print(f"Result: {success}, {message}")
        
#         # Wait for user input
#         input("Press Enter to exit...")
        
#     except KeyboardInterrupt:
#         print("Program interrupted by user")
#     finally:
#         # Clean up resources
#         stop()  # Ensure robot stops
#         shutdown()