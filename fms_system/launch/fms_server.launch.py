from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fms_system',
            executable='agent_manager_node',
            name='agent_manager'
        ),
        Node(
            package='fms_system',
            executable='task_manager_node',
            name='task_manager'
        ),
        Node(
            package='fms_system',
            executable='inventory_manager_node',
            name='inventory_manager'
        ),
        Node(
            package='fms_system',
            executable='traffic_manager_node',
            name='traffic_manager'
        ),
        Node(
            package='fms_system',
            executable='monitoring_manager_node',
            name='monitoring_manager'
        )
    ])
