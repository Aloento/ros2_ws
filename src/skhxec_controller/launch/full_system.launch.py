from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        Node(
            package='midterm_sensor_2',
            executable='sensor',
            name='sensor_node',
            output='screen'
        ),
        
        Node(
            package='skhxec_controller',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
    ])
