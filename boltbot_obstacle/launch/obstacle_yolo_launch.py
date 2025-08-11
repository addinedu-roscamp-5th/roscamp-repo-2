from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('boltbot_obstacle'), 'config', 'bridge.yaml'
    ])

    return LaunchDescription([
        Node(package='boltbot_obstacle', executable='obstacle_node', output='screen'),
        Node(package='boltbot_obstacle', executable='yolo_node', output='screen'),
        Node(package='boltbot_obstacle', executable='obstacle_decision', output='screen'),

        # ✅ '--config' 없이 파일 경로만 넘깁니다.
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge',
            output='screen',
            arguments=[bridge_yaml],
        ),
    ])
