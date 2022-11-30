from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    count_arg = LaunchConfiguration('count', default='50')
    record = LaunchConfiguration('record', default='false')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
            condition=IfCondition(record)
        ),
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='publisher_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'count': count_arg}]
        ),
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='subscriber_node',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(record)
        )
    ])
