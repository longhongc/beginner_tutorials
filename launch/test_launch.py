from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='beginner_tutorials',
            executable='beginner_tutorials_test',
            name='beginner_tutorials_test_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
