from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_control',
            executable='offboard_node',
            name='offboard_node',
            output='screen',
        ),
    ])