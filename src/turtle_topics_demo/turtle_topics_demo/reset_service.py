import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger


class MissionResetService(Node):
    def __init__(self):
        super().__init__('mission_reset_service')

        # 可重入回调组，允许同一节点内多回调并发
        self.cb_group = ReentrantCallbackGroup()

        # 对外暴露的服务：/mission/reset
        self.srv = self.create_service(
            Trigger, '/mission/reset', self.on_reset, callback_group=self.cb_group
        )

        # 内部客户端：turtlesim 的 /reset
        self.reset_cli = self.create_client(Empty, '/reset', callback_group=self.cb_group)

        self.get_logger().info('Reset Service ready: /mission/reset -> /reset')

    def call_empty_with_event(self, client, name: str, timeout: float = 3.0) -> bool:
        """不在回调内spin；用事件等待Future完成，由执行器其他线程推进Future。"""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'{name} 服务不可用')
            return False

        req = Empty.Request()
        fut = client.call_async(req)

        done = threading.Event()
        def _mark_done(_):
            done.set()
        fut.add_done_callback(_mark_done)

        # 等待Future完成（执行器会在其他线程处理通信）
        if not done.wait(timeout):
            self.get_logger().error(f'调用 {name} 超时（>{timeout}s）')
            return False

        if fut.result() is None:
            self.get_logger().error(f'调用 {name} 失败（无结果）')
            return False

        self.get_logger().info(f'已成功调用 {name}')
        return True

    def on_reset(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok = self.call_empty_with_event(self.reset_cli, '/reset', timeout=3.0)
        resp.success = bool(ok)
        resp.message = 'reset done' if ok else 'reset failed'
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = MissionResetService()

    # 多线程执行器，确保有独立线程推进客户端Future
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
