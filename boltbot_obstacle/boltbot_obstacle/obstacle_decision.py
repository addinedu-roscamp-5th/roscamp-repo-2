#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Multi-robot obstacle_decision (frame_stop → 항상 AVOID, 각속도는 상태 머신에서 처리)

입력(포트 접두, 로봇측):
  - /robot_9991|2|3/bubble_stop       (Bool)
  - /robot_9991|2|3/frame_stop        (Bool)
  - /robot_9991|2|3/obstacle/dynamic  (Bool)  ← 구독만, 미사용

출력(포트 접두, 로봇측):
  - /robot_9991|2|3/drive_state       (String: run/stop/avoid)
  - /robot_9991|2|3/obstacle_status   (String: SAFETY_STOP/AVOIDING/"")
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool

# 고정 매핑 (관제 이름 → 로봇 포트)  ※ 네임스페이스는 /robot_<port> 사용
ROBOT_MAP = [
    ('robot1', 9991),
    ('robot2', 9992),
    ('robot3', 9993),
]

class _RobotCtx:
    def __init__(self, name: str, port: int):
        self.name = name
        self.port = port
        self.ns_port = f"/robot_{port}"

        # 입력 상태
        self.bubble_stop = False
        self.frame_stop = False
        # 호환용: 구독은 하지만 로직에 사용하지 않음
        self.dynamic_obstacle = False

        # 마지막 발행값(변경시에만 퍼블리시)
        self.last_drive_state = None
        self.last_obs_status = None

        # 퍼블리셔 핸들
        self.pub_drive = None
        self.pub_obs = None


class MultiObstacleDecisionNode(Node):
    RUN   = 'run'
    STOP  = 'stop'
    AVOID = 'avoid'

    def __init__(self):
        super().__init__('multi_obstacle_decision')

        # QoS
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # 로봇 컨텍스트 등록 및 Pub/Sub 바인딩
        self.robots = []
        for name, port in ROBOT_MAP:
            ctx = _RobotCtx(name, port)

            # 퍼블리셔(포트 접두)
            ctx.pub_drive = self.create_publisher(String, f'{ctx.ns_port}/drive_state', qos)
            ctx.pub_obs   = self.create_publisher(String, f'{ctx.ns_port}/obstacle_status', qos)

            # 구독(포트 접두)
            self.create_subscription(Bool, f'{ctx.ns_port}/bubble_stop',
                                     lambda m, c=ctx: self._bubble_cb(c, m), qos)
            self.create_subscription(Bool, f'{ctx.ns_port}/frame_stop',
                                     lambda m, c=ctx: self._frame_cb(c, m), qos)
            # 호환용: 동적여부 구독(미사용)
            self.create_subscription(Bool, f'{ctx.ns_port}/obstacle/dynamic',
                                     lambda m, c=ctx: self._dynamic_cb(c, m), qos)

            self.get_logger().info(
                f'[{ctx.name} | {ctx.ns_port}] subs: '
                f'{ctx.ns_port}/{{bubble_stop,frame_stop,obstacle/dynamic}}  '
                f'pubs: {ctx.ns_port}/{{drive_state,obstacle_status}}'
            )
            self.robots.append(ctx)

        # 10Hz 주기 평가
        self.create_timer(0.1, self._evaluate_all)
        self.get_logger().info('🛞 MultiObstacleDecision started (frame_stop → AVOID only).')

    # ── 콜백(로봇별) ───────────────────────────────────────────
    def _bubble_cb(self, ctx: _RobotCtx, msg: Bool):
        ctx.bubble_stop = bool(msg.data)

    def _frame_cb(self, ctx: _RobotCtx, msg: Bool):
        ctx.frame_stop = bool(msg.data)

    def _dynamic_cb(self, ctx: _RobotCtx, msg: Bool):
        # 구독은 유지하되, 평가 로직에는 사용하지 않음
        ctx.dynamic_obstacle = bool(msg.data)

    # ── 주기 평가 ───────────────────────────────────────────────
    def _evaluate_all(self):
        for ctx in self.robots:
            # 기본값
            drive_state = self.RUN
            obs_status = ""

            # 1) 안전버블: 최우선 정지
            if ctx.bubble_stop:
                drive_state = self.STOP
                obs_status = "SAFETY_STOP"

            # 2) 카메라 프레임 정지: 동적/정적 무관하게 회피
            elif ctx.frame_stop:
                drive_state = self.AVOID
                obs_status = "AVOIDING"

            # 3) 평상시(run) → 기본값 유지
            # else: pass
            else:
                drive_state = self.RUN
                obs_status = ""

            # 변경시에만 발행
            if drive_state != ctx.last_drive_state:
                ctx.pub_drive.publish(String(data=drive_state))
                ctx.last_drive_state = drive_state
                self.get_logger().debug(f"{ctx.ns_port}/drive_state -> {drive_state}")

            if obs_status != ctx.last_obs_status:
                ctx.pub_obs.publish(String(data=obs_status))
                ctx.last_obs_status = obs_status
                if obs_status:
                    self.get_logger().debug(f"{ctx.ns_port}/obstacle_status -> {obs_status}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiObstacleDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
