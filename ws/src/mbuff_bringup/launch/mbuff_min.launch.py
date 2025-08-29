from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='msense_hat_node', executable='msense_hat', name='msense_hat'),
        Node(package='web_bridge_node', executable='web_bridge', name='web_bridge'),
        Node(package='mbot_controller_node', executable='mbot_controller', name='mbot_controller'),
    ])
