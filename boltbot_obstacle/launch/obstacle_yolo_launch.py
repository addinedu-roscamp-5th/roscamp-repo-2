from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('boltbot_obstacle'), 'config', 'bridge.yaml'
    ])

    return LaunchDescription([
        # 노트북에서 돌리는 퍼블리셔/판단 노드들은 도메인 100으로 고정
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='100'),

        Node(package='boltbot_obstacle', executable='obstacle_node', output='screen', respawn=True),
        Node(package='boltbot_obstacle', executable='yolo_node', output='screen', respawn=True),
        Node(package='boltbot_obstacle', executable='obstacle_decision', output='screen', respawn=True),

        # domain_bridge는 YAML만 넘기면 됩니다.
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            output='screen',
            arguments=[bridge_yaml],  # 필요시: + ['--ros-args', '--log-level', 'debug']
            respawn=True,
        ),
    ])
