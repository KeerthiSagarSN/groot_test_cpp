from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='groot_test_cpp',
            executable='groot_test_node',
            name='groot_test_node',
            output='screen'
        )
    ])
