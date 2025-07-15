from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bolt_fms',
            executable='agent_manager_node',
            name='agent_manager'
        ),
        Node(
            package='bolt_fms',
            executable='task_manager_node',
            name='task_manager'
        ),
        Node(
            package='bolt_fms',
            executable='inventory_manager_node',
            name='inventory_manager'
        ),
        Node(
            package='bolt_fms',
            executable='traffic_manager_node',
            name='traffic_manager'
        ),
        Node(
            package='bolt_fms',
            executable='monitoring_manager_node',
            name='monitoring_manager'
        )
    ])
