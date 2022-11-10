import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    count_arg = LaunchConfiguration('count', default='50')

    return LaunchDescription([
        Node(
            package="beginner_tutorials",
            executable="talker",
            name="publisher_node",
            output="screen",
            emulate_tty=True,
            parameters=[{'count': count_arg}]
        ),
        Node(
            package="beginner_tutorials",
            executable="listener",
            name="subscriber_node",
            output="screen",
            emulate_tty=True,
        )
    ])
