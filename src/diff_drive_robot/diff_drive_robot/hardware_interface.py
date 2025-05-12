#!/usr/bin/env python3
import os
import can
import time
import rclpy
import subprocess
from rclpy.node import Node
from diff_drive_msgs.msg import Feedback, WheelCommand
from ament_index_python.packages import get_package_share_directory
class DiffDriveHardwareInterface(Node):
    def __init__(self):
        super().__init__('diff_drive_hardware_interface')
        
        # 初始化CAN总线连接
        self.bus = can.ThreadSafeBus(interface='socketcan', channel="can0", bitrate=1000000)
        
        # 创建发布者和订阅者
        self.feedback_publisher = self.create_publisher(Feedback, 'wheel_feedback', 10)
        self.wheel_cmd_subscription = self.create_subscription(
            WheelCommand, 
            'wheel_cmd', 
            self.wheel_cmd_callback, 
            10)
            
        # 定义轮子电机ID
        self.wheel_motors = [20, 21] 

        self.speeds = []
        self.feedback_speed = [0.0,0.0]
        self.feedback_pos = [0.0,0.0]
         
        # 创建反馈消息的定时器，每100ms发送一次反馈
        self.speed_ctrl_timer = self.create_timer(0.1, self.speed_ctrl_loop)
        self.feedback_timer = self.create_timer(0.1, self.publish_feedback)
        
        self.get_logger().info("DiffDrive Hardware Interface initialized successfully!")

    def wheel_cmd_callback(self, msg):
        """处理轮子速度命令"""
        self.get_logger().debug(f"Wheel command received: left={msg.left_wheel_velocity}, right={msg.right_wheel_velocity}")
        
        # 将速度命令传递给电机控制函数
        self.speeds = [msg.left_wheel_velocity, msg.right_wheel_velocity]
        # self.control_wheel_speeds(speeds)
    
    def control_wheel_speeds(self, speeds):
        vel = []
        pos = []
        """通过CAN总线向轮子电机发送速度命令"""
        for i in range(len(self.wheel_motors)):
            # 将速度值转换为控制器需要的格式（乘以100，转换为小端4字节整数）
            _speed = int(speeds[i] * 100)
            l, m, n, o = _speed.to_bytes(4, "little", signed=True)
            
            # 构建CAN消息
            _cmd_ = [0xA2, 0x00, 0x00, 0x00, l, m, n, o]
            msg = can.Message(
                arbitration_id=self.wheel_motors[i] + 320,  # CAN ID = 电机ID + 320
                data=_cmd_,
                is_extended_id=False
            )
            
            # 发送消息并接收响应
            try:
                self.bus.send(msg)
                _response = self.bus.recv(timeout=0.01)
                if _response is None:
                    self.get_logger().warning(f"No response from motor {self.wheel_motors[i]}")
                    vel.append(None)
                    pos.append(None)
                else:
                    _vel_ = int.from_bytes(_response.data[4:6],"little",signed=True)
                    _pos_ = float(int.from_bytes(_response.data[6:8],"little",signed=True))
                    vel.append(_vel_)
                    pos.append(_pos_)                     
            except Exception as e:
                self.get_logger().error(f"Error sending command to motor {self.wheel_motors[i]}: {e}")  
        return vel,pos
    
    def speed_ctrl_loop(self):

        if self.speeds:
            print("Speed", self.speeds)
            vel,pos = self.control_wheel_speeds(self.speeds)
            self.speeds = []
        else:
            vel,pos = self.control_wheel_speeds([0.0,0.0])
            print("Stopping")

        if (not None in vel) and (not None in pos) :
            self.feedback_speed = vel
            self.feedback_pos = pos
            print(vel)
            print(pos)
        else:
            pass
        
        # pos = self.get_multiturn_data(self.wheel_motors)

        # if not None in pos:
        #     self.feedback_pos = pos
        # else:
        #     pass



    def publish_feedback(self):
        """发布轮子速度、位置和多圈位置反馈"""
        feedback = Feedback()
        
        # # 获取轮子基本反馈
        # left_pos, left_vel = self.get_motor_feedback(self.wheel_motors[0])
        # right_pos, right_vel = self.get_motor_feedback(self.wheel_motors[1])
        
        # # 获取多圈数据
        # left_multiturn = self.get_multiturn_data(self.wheel_motors[0])
        # right_multiturn = self.get_multiturn_data(self.wheel_motors[1])
        
        # 填充反馈消息
        feedback.left_wheel_position = float(self.feedback_pos[0])
        feedback.left_wheel_velocity = float(self.feedback_speed[0])
        feedback.right_wheel_position = float(self.feedback_pos[1])
        feedback.right_wheel_velocity = float(self.feedback_speed[1])
        
        # 添加多圈位置数据 
        # feedback.left_wheel_multiturn_position = (self.feedback_pos[0])  
        # feedback.right_wheel_multiturn_position = (self.feedback_pos[1]) 
        
        feedback.timestamp = self.get_clock().now().to_msg()
        
        # 发布反馈消息
        self.feedback_publisher.publish(feedback)
    
    def get_motor_feedback(self, motor_id):
        query_msg = can.Message(
            arbitration_id=motor_id + 320,
            data=[0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False
        )
        
        try:
            # 发送查询
            self.bus.send(query_msg)
            # 接收响应
            response = self.bus.recv(timeout=0.05)
            
            if response is not None:
                # 解析位置和速度 - 实际格式需要根据电机控制器的协议调整
                velocity = int.from_bytes(response.data[4:5], byteorder='little', signed=True) / 100.0
                position= int.from_bytes(response.data[6:7], byteorder='little', signed=True) / 100.0
                return position, velocity
        except Exception as e:
            self.get_logger().error(f"Error getting feedback from motor {motor_id}: {e}")
        
        # 如果出错，返回默认值
        return 0.0, 0.0
    
    def send_msg(self, bus, motor_id, cmd, need_reply=False):
        """发送CAN消息并可选择性接收响应"""
        msg = can.Message(
            arbitration_id=motor_id + 320,
            data=cmd,
            is_extended_id=False
        )
        
        try:
            bus.send(msg)
            if need_reply:
                response = bus.recv(timeout=0.05)
                return response
        except Exception as e:
            self.get_logger().error(f"Error sending message to motor {motor_id}: {e}")
        return None
    
    def get_multiturn_data(self, motor_id):
    # 这里我们需要将单个ID封装为列表
        multiturn_position = []
        
        for i in range(len(motor_id)):
            _cmd_ = [0x00]*8
            _cmd_[0] = 0x92
            recv = self.send_msg(self.bus, motor_id[i], _cmd_, True)
            if not recv == None:
                
                # 添加检查，只接收0x92开头的数据
                if recv.data[0] == 0x92:
                    pos = int.from_bytes(recv.data[4:7], "little", signed=True)
                    pos = pos/100  # gear ratio is 100
                    print(recv)
                else:
                    pos = None
            else:
                pos = None
            multiturn_position.append(pos)
            # print(multiturn_position)
    
        return multiturn_position
        
    def shutdown(self):
        """关闭CAN总线和资源"""
        if hasattr(self, 'bus') and self.bus is not None:
            self.bus.shutdown()
            self.get_logger().info("CAN bus shutdown completed")
def setup_can_interface():
    """设置CAN接口"""
    package_name = 'diff_drive_robot'
    try:
        # 获取脚本路径
        path = os.path.join(
            get_package_share_directory(package_name),
            'scripts'
        )
        
        # 运行CAN接口设置脚本
        delete_script = os.path.join(path, 'can_delete.sh')
        config_script = os.path.join(path, 'can_config.sh')
        
        # 从环境变量获取sudo密码，如果未设置则使用默认密码
        pwd = os.environ.get('SUDO_PASSWORD', '')
        
        if pwd:
            # 删除可能存在的CAN接口
            subprocess.call(f'echo {pwd} | sudo -S {delete_script}', shell=True)
            time.sleep(1)
            # 配置CAN接口
            subprocess.call(f'echo {pwd} | sudo -S {config_script}', shell=True)
            return True
        else:
            print("Warning: SUDO_PASSWORD environment variable not set")
            print("Trying to run scripts without sudo...")
            subprocess.call(delete_script, shell=True)
            time.sleep(1)
            subprocess.call(config_script, shell=True)
            return True
            
    except Exception as e:
        print(f"Error setting up CAN interface: {e}")
        return False
def main(args=None):
    # 初始化ROS节点
    rclpy.init(args=args)
    
    # 设置CAN接口
    if not setup_can_interface():
        print("Failed to setup CAN interface. Exiting...")
        rclpy.shutdown()
        return
    
    # 创建硬件接口节点
    hw_interface = DiffDriveHardwareInterface()
    
    try:
        # 运行节点直到被中断
        rclpy.spin(hw_interface)
    except KeyboardInterrupt:
        print("Node interrupted by user")
    except Exception as e:
        print(f"Error during node execution: {e}")
    finally:
        # 清理资源
        hw_interface.shutdown()
        hw_interface.destroy_node()
        rclpy.shutdown()
        print("############### DiffDrive Hardware Interface terminated ###############")
if __name__ == '__main__':
    main()