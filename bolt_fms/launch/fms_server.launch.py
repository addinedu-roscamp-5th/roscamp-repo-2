from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('bolt_fms'),
        'config',
        'domain_bridge_config.yaml'
    )

    return LaunchDescription([
        # Node(
        #     package='bolt_fms',
        #     executable='agent_manager_node',
        #     name='agent_manager'
        # ),
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
            executable='tag_tracker_node.py',
            name='tag_tracker'
        ),
        # Node(
        #     package='bolt_fms',
        #     executable='monitoring_manager_node',
        #     name='monitoring_manager'
        # ),
        # domain_bridge 노드는 ExecuteProcess로 실행해야 함
        ExecuteProcess(
            cmd=[
                '/opt/ros/jazzy/lib/domain_bridge/domain_bridge',
                config_dir  # ✅ 올바르게 수정된 변수
            ],
            output='screen'
        )
    ])
