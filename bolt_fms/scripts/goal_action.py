import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class TopicToActionBridge(Node):
    def __init__(self):
        super().__init__('topic_to_action_bridge')
        # 1) /goal_pose 구독
        self.sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        # 2) NavigateToPose 액션 클라이언트
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_callback(self, pose_msg: PoseStamped):
        # 이미 요청 중이면 무시
        if self.action_client.wait_for_server(timeout_sec=5.0) is False:
            self.get_logger().error('액션 서버 연결 실패')
            return

        # 액션 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info(f'토픽 → 액션으로 변환: x={pose_msg.pose.position.x:.2f}, '
                               f'y={pose_msg.pose.position.y:.2f}')

        # 비동기로 전송
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('액션 요청이 거부되었습니다')
            return
        self.get_logger().info('액션 서버가 목표를 수락했습니다. 피드백 대기...')
        # 피드백/결과 콜백 등록
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # ABORTED
            self.get_logger().error('주행이 실패했습니다.')
        else:
            self.get_logger().info('주행 완료!')

def main(args=None):
    rclpy.init(args=args)
    node = TopicToActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()