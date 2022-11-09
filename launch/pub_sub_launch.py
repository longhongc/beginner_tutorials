from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="beginner_tutorials",
            executable="talker",
            name="publisher_node",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="beginner_tutorials",
            executable="listener",
            name="subscriber_node",
            output="screen",
            emulate_tty=True,
        )
    ])
