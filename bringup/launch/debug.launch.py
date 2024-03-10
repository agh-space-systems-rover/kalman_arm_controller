from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kalman_arm_controller',
            executable='can_read_test',
            output='screen',
            emulate_tty=True,
            name='can_read_test'
        )
    ])