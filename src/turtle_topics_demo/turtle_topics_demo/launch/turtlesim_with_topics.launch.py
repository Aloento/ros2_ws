"""
turtlesim_with_topics.launch.py
一次性启动：
  - turtlesim_node
  - vel_publisher（可通过参数调速）
  - pose_subscriber
  - cmdvel_subscriber
用法：
  ros2 launch turtle_topics_demo turtlesim_with_topics.launch.py
可传参：
  ros2 launch turtle_topics_demo turtlesim_with_topics.launch.py linear_speed:=2.0 angular_speed:=1.0 Hz:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed_arg  = DeclareLaunchArgument('linear_speed',  default_value='1.5')
    angular_speed_arg = DeclareLaunchArgument('angular_speed', default_value='1.5')
    hz_arg            = DeclareLaunchArgument('Hz',            default_value='10.0')

    return LaunchDescription([
        linear_speed_arg, angular_speed_arg, hz_arg,

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='turtle_topics_demo',
            executable='vel_pub',
            name='vel_pub',
            parameters=[{
                'linear_speed':  LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'Hz':            LaunchConfiguration('Hz'),
            }]
        ),

        Node(
            package='turtle_topics_demo',
            executable='pose_sub',
            name='pose_sub'
        ),

        Node(
            package='turtle_topics_demo',
            executable='cmdvel_echo',
            name='cmdvel_echo'
        ),
    ])
