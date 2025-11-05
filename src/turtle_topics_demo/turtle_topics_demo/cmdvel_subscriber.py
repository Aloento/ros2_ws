"""
cmdvel_subscriber.py
订阅 /turtle1/cmd_vel（geometry_msgs/msg/Twist），打印控制指令，方便在 rqt_graph 之外做数据核对。
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelEcho(Node):
    def __init__(self):
        super().__init__('cmdvel_echo')
        self.sub = self.create_subscription(Twist, '/turtle1/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        self.get_logger().info(
            f'cmd_vel: lin=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) '
            f'ang=({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelEcho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
