from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='boltbot_obstacle', executable='obstacle_node', output='screen'),
        Node(package='boltbot_obstacle', executable='yolo_node', output='screen'),
    ])