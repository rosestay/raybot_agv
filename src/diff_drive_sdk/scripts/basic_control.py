#!/usr/bin/env python3
"""
基本控制命令行工具
===============

提供命令行接口来控制机器人的基本移动。
"""

import rclpy
import argparse
import time
import sys
from diff_drive_sdk import DiffDriveRobot

def main(args=None):
    """主函数，处理命令行参数并控制机器人"""
    parser = argparse.ArgumentParser(description='差速驱动机器人基本控制')
    
    # 添加命令行参数
    parser.add_argument('command', choices=['forward', 'backward', 'left', 'right', 'stop', 'wheels', 'info'],
                      help='控制命令')
    parser.add_argument('--speed', type=float, default=0.3,
                      help='速度(m/s或rad/s)')
    parser.add_argument('--time', type=float, default=2.0,
                      help='运行时间(秒)')
    parser.add_argument('--left', type=float,
                      help='左轮速度(仅用于wheels命令)')
    parser.add_argument('--right', type=float,
                      help='右轮速度(仅用于wheels命令)')
    
    # 解析命令行参数
    parsed_args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init(args=args)
    
    # 创建机器人控制对象
    robot = DiffDriveRobot()
    
    try:
        # ... [命令处理逻辑] ...
    
    except KeyboardInterrupt:
        print("\n操作被用户中断")
    
    finally:
        # 确保机器人停止
        robot.stop()
        robot.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())