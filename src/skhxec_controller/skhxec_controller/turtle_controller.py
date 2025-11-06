#!/usr/bin/env python3
"""
Turtle Controller Node
订阅传感器数据并控制TurtleSim机器人
- 负数 -> 向前移动1.0单位
- 正奇数 -> 向右转90度
- 正偶数 -> 向左转90度
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # 订阅传感器话题
        self.subscription = self.create_subscription(
            Int32,
            'sensor2_signal',
            self.sensor_callback,
            10
        )
        
        # 发布到TurtleSim的速度话题
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        # 用于停止运动的timer
        self.stop_timer = None
        
        self.get_logger().info('Turtle Controller节点已启动')
        self.get_logger().info('订阅话题: sensor2_signal')
        self.get_logger().info('发布话题: /turtle1/cmd_vel')
    
    def sensor_callback(self, msg):
        """处理传感器数据的回调函数"""
        sensor_value = msg.data
        cmd_vel = Twist()
        
        if sensor_value < 0:
            # 负数 -> 向前移动
            cmd_vel.linear.x = 1.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info(f'传感器值: {sensor_value} (负数) -> 向前移动1.0单位')
            
        elif sensor_value > 0 and sensor_value % 2 == 1:
            # 正奇数 -> 向右转90度 (负角速度)
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -math.pi / 2  # -90度转为弧度
            self.get_logger().info(f'传感器值: {sensor_value} (正奇数) -> 向右转90度')
            
        elif sensor_value > 0 and sensor_value % 2 == 0:
            # 正偶数 -> 向左转90度 (正角速度)
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = math.pi / 2  # 90度转为弧度
            self.get_logger().info(f'传感器值: {sensor_value} (正偶数) -> 向左转90度')
        
        # 发布速度命令
        self.publisher.publish(cmd_vel)
        
        # 取消之前的停止timer(如果存在)
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        
        # 创建新的停止timer
        self.stop_timer = self.create_timer(1.0, self.stop_turtle_callback)
    
    def stop_turtle_callback(self):
        """停止turtle的移动"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)
        
        # 取消timer,避免重复执行
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
