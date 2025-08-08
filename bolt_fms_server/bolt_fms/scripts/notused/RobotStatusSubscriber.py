import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState

class RobotStatusSubscriber(Node):
    """여러 로봇의 상태를 다양한 토픽에서 동시 구독 및 저장"""

    def __init__(self, robot_ids):
        super().__init__('robot_status_subscriber')
        self.robot_ids = robot_ids

        # {robot_id: {토픽명: 상태}}
        self.robot_statuses = {
            robot_id: {
                'camera_pose': None,
                'target_pose': None,
                'battery': None,
                'task': None,
                'state': None,
                'robot_type': None,
                'joint_states': None,
            } for robot_id in robot_ids
        }

        self.subscribers = []

        # --- 각 토픽별로 구독 생성 ---
        for robot_id in robot_ids:
            # 위치
            self.subscribers.append(self.create_subscription(
                PoseStamped, f'/robot{robot_id}/camera_pose',
                lambda msg, rid=robot_id: self._cb('camera_pose', msg, rid), 10))
            # 목표 위치
            self.subscribers.append(self.create_subscription(
                PoseStamped, f'/robot{robot_id}/target_pose',
                lambda msg, rid=robot_id: self._cb('target_pose', msg, rid), 10))
            # 배터리
            self.subscribers.append(self.create_subscription(
                Float32, f'/robot{robot_id}/battery',
                lambda msg, rid=robot_id: self._cb('battery', msg, rid), 10))
            # 작업
            self.subscribers.append(self.create_subscription(
                String, f'/robot{robot_id}/task',
                lambda msg, rid=robot_id: self._cb('task', msg, rid), 10))
            # 상태
            self.subscribers.append(self.create_subscription(
                String, f'/robot{robot_id}/state',
                lambda msg, rid=robot_id: self._cb('state', msg, rid), 10))
            # 타입
            self.subscribers.append(self.create_subscription(
                String, f'/robot{robot_id}/robot_type',
                lambda msg, rid=robot_id: self._cb('robot_type', msg, rid), 10))
            # 조인트 상태
            self.subscribers.append(self.create_subscription(
                JointState, f'/robot{robot_id}/joint_states',
                lambda msg, rid=robot_id: self._cb('joint_states', msg, rid), 10))

            self.get_logger().info(f'로봇{robot_id}의 모든 상태 토픽 구독 시작!')

    def _cb(self, key, msg, robot_id):
        """공통 콜백: 상태 저장 + 로그"""
        self.robot_statuses[robot_id][key] = msg
        # 상태에 따라 다르게 출력(간단 요약만)
        if key == 'camera_pose' and msg is not None:
            pos = msg.pose.position
            self.get_logger().info(f'[로봇{robot_id}] 위치: ({pos.x:.2f}, {pos.y:.2f})')
        elif key == 'battery' and msg is not None:
            self.get_logger().info(f'[로봇{robot_id}] 배터리: {msg.data:.1f}')
        elif key in ['task', 'state', 'robot_type'] and msg is not None:
            self.get_logger().info(f'[로봇{robot_id}] {key}: {msg.data}')
        elif key == 'joint_states' and msg is not None:
            self.get_logger().info(f'[로봇{robot_id}] 조인트 상태: {msg.name} {msg.position}')

    def get_robot_status(self, robot_id):
        """특정 로봇의 모든 최신 상태 딕셔너리 반환"""
        return self.robot_statuses.get(robot_id, None)

def main(args=None):
    rclpy.init(args=args)
    robot_ids = [1, 2, 3]  # 원하는 로봇 ID 지정
    node = RobotStatusSubscriber(robot_ids)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
