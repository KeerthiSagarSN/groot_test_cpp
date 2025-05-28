from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='groot_test_cpp',
            executable='nav2_zmq_bridge',
            name='nav2_zmq_bridge',
            output='screen'
        )
    ])
