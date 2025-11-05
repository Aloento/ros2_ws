"""
vel_publisher.py
持续向 /turtle1/cmd_vel 发布速度指令，让乌龟做圆周运动。
可调参数（ros2 param 或 Launch 里设置）：
  - linear_speed  (float, 默认 1.5)
  - angular_speed (float, 默认 1.5)
  - Hz            (float, 默认 10.0)   发布频率
日志：每秒打印一次当前发布的线/角速度。
"""


import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleVelPublisher(Node):
    def __init__(self):
        super().__init__('turtle_vel_publisher')

        # 声明参数（可在运行时覆盖）
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('Hz', 10.0)

        # 读取参数
        self.lin = float(self.get_parameter('linear_speed').value) # type: ignore
        self.ang = float(self.get_parameter('angular_speed').value) # type: ignore
        self.hz  = float(self.get_parameter('Hz').value) # type: ignore

        # 创建 Publisher，话题名与类型要与 turtlesim 匹配
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 发布定时器
        period = 1.0 / max(self.hz, 1.0)
        self.timer = self.create_timer(period, self.tick)

        # 每秒打印一次日志（不然 10Hz 下太刷屏）
        self.print_timer = self.create_timer(1.0, self.print_once)

        self.get_logger().info(
            f'Vel Publisher 启动：lin={self.lin:.2f}, ang={self.ang:.2f}, Hz={self.hz:.1f}'
        )

    def tick(self):
        """周期性发布 Twist"""
        msg = Twist()
        msg.linear.x  = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)

    def print_once(self):
        """低频打印，便于终端阅读"""
        self.get_logger().info(f'Publishing cmd_vel: lin={self.lin:.2f}, ang={self.ang:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
