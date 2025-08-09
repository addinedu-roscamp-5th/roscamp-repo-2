#!/usr/bin/env python3
"""
obstacle_decision.py

장애물 판단 및 목표 도달 상태를 통합하여 주행 상태(`/drive_state`)와
관제용 임무 상태(`/task_status`)를 발행하는 노드입니다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

class ObstacleDecisionNode(Node):
    # 상태 문자열 정의
    RUN   = 'run'
    STOP  = 'stop'
    AVOID = 'avoid'

    def __init__(self):
        super().__init__('obstacle_decision')

        # 내부 상태 플래그 초기화
        self.bubble_stop = False
        self.frame_stop = False
        self.dynamic_obstacle = False
        self.is_avoiding = False
        self.goal_reached = False
        self.has_goal = False

        # 구독자(Subscriber) 설정
        self.create_subscription(PoseStamped, '/target_pose', self.goal_callback, 10)
        self.create_subscription(Bool, '/bubble_stop',      self.bubble_callback,   10)
        self.create_subscription(Bool, '/frame_stop',       self.frame_callback,    10)
        self.create_subscription(Bool, '/obstacle/dynamic', self.dynamic_callback,  10)
        self.create_subscription(String, '/state',          self.state_callback,    10)

        # 발행자(Publisher) 설정
        self.drive_state_pub = self.create_publisher(String, '/drive_state', 10)
        
        # ✅ 추가: 관제 시스템 보고용 '임무 상태' 퍼블리셔
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)
        
        self.get_logger().info('🛞 Obstacle Decision Node 시작')

        # 주기적으로 evaluate() 호출하여 상태 갱신
        self.create_timer(0.1, self.evaluate)

    def goal_callback(self, msg: PoseStamped):
        # 새로운 목표를 받으면 상태 플래그 초기화
        self.has_goal = True
        self.goal_reached = False
        self.is_avoiding = False
        self.get_logger().info('Received new target_pose, resetting flags.')
        self.evaluate()

    def bubble_callback(self, msg: Bool):
        self.bubble_stop = msg.data
        self.evaluate()

    def frame_callback(self, msg: Bool):
        self.frame_stop = msg.data
        if self.frame_stop:
            self.is_avoiding = False
        self.evaluate()

    def dynamic_callback(self, msg: Bool):
        if self.bubble_stop or self.frame_stop:
            self.dynamic_obstacle = msg.data
        self.evaluate()

    def state_callback(self, msg: String):
        self.goal_reached = (msg.data == 'GoalReached')
        if self.goal_reached:
            self.has_goal = False
        self.evaluate()

    def evaluate(self):
        drive_state = self.RUN # 기본 상태는 주행
        task_status = ""     # 관제 보고용 상태
        
        # --- 의사결정 로직 (우선순위가 높은 순서대로) ---
        # 1) 목표 도달 또는 목표 없음
        if self.goal_reached or not self.has_goal:
            drive_state = self.STOP
            # task_status는 RobotGoalController가 SUCCEEDED 또는 IDLE로 발행하므로 여기서 발행 안 함
        
        # 2) 라이다 안전 버블 탐지 시 무조건 정지
        elif self.bubble_stop:
            drive_state = self.STOP
            task_status = "SAFETY_STOP" # ✅ 관제에 안전 정지 상태 보고
        
        # 3) 카메라 영상 기준 장애물 판단
        elif self.frame_stop:
            if self.dynamic_obstacle: # 동적 장애물이면 정지
                drive_state = self.STOP
                task_status = "BLOCKED" # ✅ 관제에 동적 장애물로 막혔다고 보고
            elif not self.is_avoiding: # 정적 장애물이고 아직 회피 시작 안했으면 회피
                self.is_avoiding = True
                drive_state = self.AVOID
            else: # 이미 회피 중이면 계속 주행
                drive_state = self.RUN
        
        # 4) 그 외 모든 경우는 정상 주행
        else:
            drive_state = self.RUN

        # 최종 결정된 주행 상태 발행
        self.drive_state_pub.publish(String(data=drive_state))

        # ✅ 추가: 관제 보고용 상태가 결정되었을 경우에만 발행
        if task_status:
            self.task_status_pub.publish(String(data=task_status))
            
        self.get_logger().debug(f"Published drive_state: {drive_state}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()