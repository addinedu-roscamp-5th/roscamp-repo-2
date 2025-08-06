#!/usr/bin/env python3
"""
obstacle_decision.py

장애물 판단 결과를 하나의 토픽(`/drive_cmd`)으로 통합하여
주행(RUN), 정지(STOP), 회피(AVOID) 상태를 발행하는 노드입니다.

State codes:
  0 → RUN
  1 → STOP
  2 → AVOID
"""

import rclpy
from rclpy.node    import Node
from std_msgs.msg  import Int8, Bool

class ObstacleDecisionNode(Node):
    RUN   = 0
    STOP  = 1
    AVOID = 2

    def __init__(self):
        super().__init__('obstacle_decision')

        self.bubble   = False
        self.frame    = False
        self.dynamic  = False
        self.avoiding = False

        # 구독: 안전버블, 카메라 정지, 동·정판별
        self.create_subscription(Bool, '/bubble_stop',      self.bubble_cb,   10)
        self.create_subscription(Bool, '/frame_stop',       self.frame_cb,    10)
        self.create_subscription(Bool, '/obstacle/dynamic', self.dynamic_cb,  10)

        # 퍼블리시: 주행 상태(RUN/STOP/AVOID)
        self.cmd_pub = self.create_publisher(Int8, '/drive_cmd', 10)

    def bubble_cb(self, msg: Bool):
        self.bubble = msg.data
        self.evaluate()

    def frame_cb(self, msg: Bool):
        # 카메라 기준 정지 신호
        self.frame = msg.data
        if self.frame:
            # 정지 모드 진입 시 회피 플래그 리셋
            self.avoiding = False
        self.evaluate()

    def dynamic_cb(self, msg: Bool):
        # 동정 플래그는 정지 중일 때만 업데이트
        if self.bubble or self.frame:
            self.dynamic = msg.data
        self.evaluate()

    def evaluate(self):
        # 1) 안전버블 진입 시 무조건 정지
        if self.bubble:
            state = self.STOP

        # 2) 카메라 기준 정지 상태
        elif self.frame:
            # (a) 정적 장애물 & 아직 회피 안 함 → AVOID
            if not self.dynamic and not self.avoiding:
                self.avoiding = True
                self.get_logger().info('Static obstacle → AVOIDING')
                state = self.AVOID

            # (b) 동적 장애물 → 계속 STOP
            elif self.dynamic:
                state = self.STOP

            # (c) 이미 회피 중 → RUN
            else:
                state = self.RUN

        # 3) 그 외 정상 주행
        else:
            state = self.RUN

        # 퍼블리시
        self.cmd_pub.publish(Int8(data=state))

    def destroy_node(self):
        # Node 종료 시 마지막으로 RUN 상태를 퍼블리시
        self.cmd_pub.publish(Int8(data=self.RUN))
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
