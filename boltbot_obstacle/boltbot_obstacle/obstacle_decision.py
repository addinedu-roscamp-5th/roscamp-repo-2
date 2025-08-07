#!/usr/bin/env python3
"""
obstacle_decision.py

장애물 판단 및 목표 도달 상태를 하나의 토픽(`/drive_state`)으로 통합하여
주행(run), 정지(stop), 회피(avoid) 상태를 발행하는 노드입니다.

State values:
  'run'   → RUN
  'stop'  → STOP
  'avoid' → AVOID
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
        self.bubble = False
        self.frame = False
        self.dynamic = False
        self.avoiding = False
        self.goal_reached = False
        self.has_goal = False

        # 구독: 목표 위치, 안전버블, 카메라 정지, 동/정 판별, 상태 머신 상태
        self.create_subscription(PoseStamped, '/target_pose', self.goal_cb, 10)
        self.create_subscription(Bool, '/bubble_stop',      self.bubble_cb,   10)
        self.create_subscription(Bool, '/frame_stop',       self.frame_cb,    10)
        self.create_subscription(Bool, '/obstacle/dynamic', self.dynamic_cb,  10)
        self.create_subscription(String, '/state',          self.state_cb,    10)

        # 퍼블리시: 주행 상태(run/stop/avoid)
        self.state_pub = self.create_publisher(String, '/drive_state', 10)
        self.get_logger().info('🛞 /drive_state 퍼블리셔 시작')

        # 주기적으로 evaluate() 호출하여 상태 갱신
        self.create_timer(0.1, self.evaluate)

    def goal_cb(self, msg: PoseStamped):
        # 목표 위치 수신 시 플래그 초기화
        self.has_goal = True
        self.goal_reached = False
        self.avoiding = False
        self.dynamic = False
        self.get_logger().info('Received new target_pose, resetting flags')
        self.evaluate()

    def bubble_cb(self, msg: Bool):
        self.bubble = msg.data
        self.evaluate()

    def frame_cb(self, msg: Bool):
        self.frame = msg.data
        if self.frame:
            self.avoiding = False
        self.evaluate()

    def dynamic_cb(self, msg: Bool):
        if self.bubble or self.frame:
            self.dynamic = msg.data
        self.evaluate()

    def state_cb(self, msg: String):
        # 상태 머신으로부터 GoalReached 신호 수신
        self.goal_reached = (msg.data == 'GoalReached')
        self.evaluate()

    def evaluate(self):
        # 0) 목표 정보 없으면 정지
        if not self.has_goal:
            state = self.STOP
        # 1) 목표 도달 시 정지
        elif self.goal_reached:
            state = self.STOP
        # 2) 안전버블 탐지 시 정지
        elif self.bubble:
            state = self.STOP
        # 3) 카메라 기준 정지 상태
        elif self.frame:
            # (a) 정적 장애물 & 미회피: 회피
            if not self.dynamic and not self.avoiding:
                self.avoiding = True
                self.get_logger().info('Static obstacle → AVOIDING')
                state = self.AVOID
            # (b) 동적 장애물: 정지
            elif self.dynamic:
                state = self.STOP
            # (c) 이미 회피 중: 주행
            else:
                state = self.RUN
        # 4) 정상 주행
        else:
            state = self.RUN

        # 상태 발행
        self.state_pub.publish(String(data=state))
        self.get_logger().debug(f"Published drive_state: {state}")

    def destroy_node(self):
        # 노드 종료 시 최종적으로 RUN 상태 발행
        self.state_pub.publish(String(data=self.RUN))
        super().destroy_node()


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
