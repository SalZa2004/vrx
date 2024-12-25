from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_gz',
            node_name='waypoint_tracker.py',
            name='waypoint_tracker',
            output='screen',
        ),
    ])
