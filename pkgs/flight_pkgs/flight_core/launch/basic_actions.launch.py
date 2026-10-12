"""Single-vehicle action and waypoint controller."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='flight_core', executable='flight_actions_node',
             name='flight_core', output='screen'),
    ])
