"""
publisher.py
--------------
这是一个最小的 ROS2 发布者节点示例。
功能：
  - 每 0.5 秒向话题 /chatter 发布一条字符串消息。
  - 消息类型：std_msgs/msg/String
"""

import rclpy                               # ROS2 Python 客户端库
from rclpy.node import Node                # Node 类是所有 ROS2 节点的基类
from std_msgs.msg import String            # 导入标准字符串消息类型


class MinimalPublisher(Node):
    """一个最小的发布者节点类"""

    def __init__(self):
        # 初始化父类构造函数，并命名节点为 "minimal_publisher"
        super().__init__('minimal_publisher')

        # 创建发布者对象
        # 参数：
        #   - 消息类型：String
        #   - 话题名称：'chatter'
        #   - 队列大小：10（缓存最近10条消息）
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # 定时器周期：每 0.5 秒调用一次回调函数
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 计数器，用来生成不同的消息
        self.i = 0

        self.get_logger().info("Publisher 节点已启动，开始发布消息到 /chatter")

    def timer_callback(self):
        """定时器回调函数：构建消息并发布"""
        msg = String()                      # 创建消息对象
        msg.data = f'Hello ROS2 #{self.i}'  # 填充数据
        self.publisher_.publish(msg)        # 发布消息
        # 打印日志方便观察
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1                         # 增加计数


def main(args=None):
    """节点主入口函数"""
    rclpy.init(args=args)                   # 初始化 ROS2 通信
    node = MinimalPublisher()               # 创建节点实例
    rclpy.spin(node)                        # 保持运行（监听事件循环）
    node.destroy_node()                     # 程序结束时销毁节点
    rclpy.shutdown()                        # 关闭 ROS2 客户端


if __name__ == '__main__':
    main()
