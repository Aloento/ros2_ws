"""
rotate_action_client.py
作为动作客户端，向 /turtle1/rotate_absolute (turtlesim/action/RotateAbsolute) 发送目标角度。
特点：
  - 通过参数 theta（弧度）设定目标角度，默认 3.14（约180度）
  - 打印反馈（当前误差）与最终结果
用法：
  ros2 run turtle_topics_demo rot_client --ros-args -p theta:=1.57
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.action import RotateAbsolute  # turtlesim 自带的 Action 定义


class RotateClient(Node):
    def __init__(self):
        super().__init__('rotate_action_client')

        # 声明参数：目标角度（弧度）
        self.declare_parameter('theta', 3.14)
        self.theta = float(self.get_parameter('theta').value) # type: ignore

        # 创建动作客户端，目标是 turtlesim 的 rotate_absolute
        self.ac = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')

        self.get_logger().info(f'动作客户端启动，准备将乌龟旋转到 theta={self.theta:.2f} rad')

        # 等待服务器并发送目标
        self.timer = self.create_timer(0.1, self.try_send_goal)
        self.goal_sent = False

    def try_send_goal(self):
        if self.goal_sent:
            return
        if not self.ac.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('等待动作服务器中...')
            return

        goal = RotateAbsolute.Goal()
        goal.theta = self.theta

        self.get_logger().info('动作服务器就绪，发送目标...')
        send_future = self.ac.send_goal_async(
            goal,
            feedback_callback=self.on_feedback
        )
        send_future.add_done_callback(self.on_goal_response)
        self.goal_sent = True

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('动作目标被拒绝')
            rclpy.shutdown()
            return
        self.get_logger().info('动作目标已接受，等待结果...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # turtlesim 的反馈包含 remaining（剩余角度误差）
        self.get_logger().info(f'[反馈] remaining = {fb.remaining:.3f} rad')

    def on_result(self, future):
        res = future.result().result
        # turtlesim 的结果一般是空的，但我们保持通用写法
        self.get_logger().info('[结果] 旋转完成！')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RotateClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
