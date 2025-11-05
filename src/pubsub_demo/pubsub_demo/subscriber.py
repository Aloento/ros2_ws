"""
subscriber.py
--------------
这是一个最小的 ROS2 订阅者节点示例。
功能：
  - 订阅话题 /chatter
  - 每当接收到 String 类型的消息时打印出来
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """一个最小的订阅者节点类"""

    def __init__(self):
        super().__init__('minimal_subscriber')

        # 创建订阅者对象
        # 参数：
        #   - 消息类型：String
        #   - 话题名称：'chatter'
        #   - 回调函数：收到消息时调用
        #   - 队列大小：10
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        # 防止 Python 的 lint 报未使用警告
        self.subscription

        self.get_logger().info("Subscriber 节点已启动，监听 /chatter 话题")

    def listener_callback(self, msg):
        """回调函数：接收到消息时调用"""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """节点主入口函数"""
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
