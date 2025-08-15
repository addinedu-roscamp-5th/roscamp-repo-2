#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import math
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class AutoMultiSafetyBubbleNode(Node):
    """
    자동 멀티 로봇 안전 버블 노드.
    - 입력:  /robot_<port>/scan (LaserScan)   ← 브리지로 들어오면 자동 감지
    - 출력:  /robot_<port>/bubble_stop (Bool)
    파라미터:
      - bubble_radius: float = 0.10 (m)
      - scan_discovery_period: float = 1.0 (s)  # 토픽 그래프 탐색 주기
    """
    SCAN_REGEX = re.compile(r"^/robot_(\d+)/scan$")
    LASER_TYPE = "sensor_msgs/msg/LaserScan"

    def __init__(self):
        super().__init__("auto_multi_safety_bubble_node")

        # Params
        self.declare_parameter("bubble_radius", 0.05)
        self.declare_parameter("scan_discovery_period", 1.0)
        self.bubble_radius = float(self.get_parameter("bubble_radius").value)
        self.discovery_period = float(self.get_parameter("scan_discovery_period").value)

        # QoS (일반적인 LaserScan/Bool 권장 값)
        self.qos_scan = QoSProfile(depth=10)
        self.qos_scan.history = HistoryPolicy.KEEP_LAST
        self.qos_scan.reliability = ReliabilityPolicy.BEST_EFFORT
        self.qos_scan.durability = DurabilityPolicy.VOLATILE

        self.qos_cmd = QoSProfile(depth=1)
        self.qos_cmd.history = HistoryPolicy.KEEP_LAST
        self.qos_cmd.reliability = ReliabilityPolicy.RELIABLE
        self.qos_cmd.durability = DurabilityPolicy.VOLATILE

        # State per robot port
        self.prev_stop = {}        # port -> bool
        self.pubs = {}             # port -> Publisher
        self.subs = {}             # port -> Subscription

        # Periodic discovery
        self.create_timer(self.discovery_period, self._discover_scans)

        self.get_logger().info(
            f"AutoMultiSafetyBubbleNode started | radius={self.bubble_radius:.2f} m, "
            f"discover every {self.discovery_period:.1f}s"
        )

        # Initial pass
        self._discover_scans()

    def _discover_scans(self):
        """주기적으로 토픽 그래프를 스캔해 /robot_<port>/scan 을 자동 등록."""
        try:
            topics = self.get_topic_names_and_types()
        except Exception as e:
            self.get_logger().warn(f"get_topic_names_and_types failed: {e}")
            return

        for name, types in topics:
            m = self.SCAN_REGEX.match(name)
            if not m:
                continue
            if self.LASER_TYPE not in types:
                continue

            port = int(m.group(1))
            if port in self.subs:
                continue  # already bound

            prefix = f"/robot_{port}"
            scan_topic = f"{prefix}/scan"
            stop_topic = f"{prefix}/bubble_stop"

            pub = self.create_publisher(Bool, stop_topic, self.qos_cmd)
            self.pubs[port] = pub
            self.prev_stop[port] = False

            sub = self.create_subscription(
                LaserScan, scan_topic, partial(self._scan_cb, port), self.qos_scan
            )
            self.subs[port] = sub

            self.get_logger().info(
                f"[{prefix}] bound: {scan_topic}  →  {stop_topic}"
            )

    def _scan_cb(self, port: int, msg: LaserScan):
        """각 로봇 포트의 스캔 콜백."""
        # NaN/Inf 제외하고 최소 거리 계산
        min_dist = math.inf
        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r):
                if r < min_dist:
                    min_dist = r

        stop = (min_dist <= self.bubble_radius)

        prev = self.prev_stop.get(port, False)
        if stop and not prev:
            self.get_logger().warn(f"[robot_{port}] Obstacle entered: {min_dist:.2f} m")
        elif not stop and prev:
            self.get_logger().info(f"[robot_{port}] Cleared: {min_dist:.2f} m")

        self.prev_stop[port] = stop
        self.pubs[port].publish(Bool(data=stop))

        if (not stop) and (min_dist is not math.inf):
            self.get_logger().debug(f"[robot_{port}] min_dist={min_dist:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = AutoMultiSafetyBubbleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
