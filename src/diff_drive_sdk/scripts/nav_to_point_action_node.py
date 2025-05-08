#!/usr/bin/env python3
"""
导航到点位Action服务节点
======================

启动导航到点位Action服务的独立节点。
"""

import rclpy
from diff_drive_sdk.nav_to_point_action import NavToPointNode

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = NavToPointNode()
    
    try:
        node.get_logger().info('开始运行导航到点位节点...')
        # 打印所有可用导航点，用于参考
        node.print_available_points()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，正在关闭节点...')
        node.cancel_navigation()  # 确保取消任何正在进行的导航
    except Exception as e:
        node.get_logger().error(f'节点运行异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()