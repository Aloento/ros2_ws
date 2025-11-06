import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        self.subscription = self.create_subscription(
            Int32,
            'sensor2_signal',
            self.sensor_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        self.stop_timer = None
        
        self.get_logger().info('Turtle Controller Node Started')
        self.get_logger().info('Subscribed to topic: sensor2_signal')
        self.get_logger().info('Publishing to topic: /turtle1/cmd_vel')
    
    def sensor_callback(self, msg):
        sensor_value = msg.data
        cmd_vel = Twist()
        
        if sensor_value < 0:
            cmd_vel.linear.x = 1.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info(f'Sensor value: {sensor_value} (negative) -> move forward 1.0 unit')
            
        elif sensor_value > 0 and sensor_value % 2 == 1:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -math.pi / 2
            self.get_logger().info(f'Sensor value: {sensor_value} (positive odd) -> turn right 90 degrees')
            
        elif sensor_value > 0 and sensor_value % 2 == 0:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = math.pi / 2
            self.get_logger().info(f'Sensor value: {sensor_value} (positive even) -> turn left 90 degrees')
        
        self.publisher.publish(cmd_vel)
        
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        
        self.stop_timer = self.create_timer(1.0, self.stop_turtle_callback)
    
    def stop_turtle_callback(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher.publish(stop_cmd)
        
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
