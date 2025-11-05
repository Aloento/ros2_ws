"""
pose_subscriber.py
订阅 /turtle1/pose（turtlesim/msg/Pose），打印乌龟位置(x,y)、朝向(theta)与线速度。
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  # 注意：turtlesim 自带的 Pose 类型


class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.cb, 10)
        self.last_t = self.get_clock().now()

    def cb(self, msg: Pose):
        # 控制一下日志频率：约 5 Hz 输出一次
        now = self.get_clock().now()
        if (now - self.last_t).nanoseconds < 200_000_000:  # 0.2 s
            return
        self.last_t = now

        self.get_logger().info(
            f'Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}, v={msg.linear_velocity:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
