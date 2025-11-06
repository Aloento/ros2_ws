#!/usr/bin/env python3
"""
Launch文件 - 启动完整的传感器控制系统
包括: TurtleSim, 传感器节点, 控制器节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动TurtleSim节点
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # 启动传感器节点
        Node(
            package='midterm_sensor_2',
            executable='sensor',
            name='sensor_node',
            output='screen'
        ),
        
        # 启动控制器节点
        Node(
            package='skhxec_controller',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
    ])
