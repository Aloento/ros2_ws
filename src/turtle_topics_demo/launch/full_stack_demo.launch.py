"""
full_stack_demo.launch.py
一键启动：
  - turtlesim_node
  - vel_publisher（圆周运动，参数可调）
  - pose_subscriber（位姿回显）
  - cmdvel_subscriber（指令回显）
  - reset_service（提供 /mission/reset）
  - rotate_action_client（立即发送一个旋转动作，可通过参数 theta 调整）
用法：
  ros2 launch turtle_topics_demo full_stack_demo.launch.py
可选参数：
  linear_speed, angular_speed, Hz, theta
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed  = DeclareLaunchArgument('linear_speed',  default_value='1.5')
    angular_speed = DeclareLaunchArgument('angular_speed', default_value='1.0')
    hz            = DeclareLaunchArgument('Hz',            default_value='10.0')
    theta         = DeclareLaunchArgument('theta',         default_value='2.6')  # 约149度

    return LaunchDescription([
        linear_speed, angular_speed, hz, theta,

        Node(package='turtlesim', executable='turtlesim_node', name='sim'),

        Node(
            package='turtle_topics_demo', executable='vel_pub', name='vel_pub',
            parameters=[{
                'linear_speed':  LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'Hz':            LaunchConfiguration('Hz'),
            }]
        ),

        Node(package='turtle_topics_demo', executable='pose_sub',    name='pose_sub'),
        Node(package='turtle_topics_demo', executable='cmdvel_echo', name='cmdvel_echo'),

        Node(package='turtle_topics_demo', executable='reset_srv',   name='reset_srv'),

        # 动作客户端：启动即发目标（便于演示；真实项目可按需触发）
        Node(
            package='turtle_topics_demo', executable='rot_client', name='rot_client',
            parameters=[{'theta': LaunchConfiguration('theta')}]
        ),
    ])
