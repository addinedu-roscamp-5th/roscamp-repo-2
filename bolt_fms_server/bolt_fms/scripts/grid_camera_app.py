#!/usr/bin/env python3
"""
그리드 카메라 장애물 앱
- 카메라 영상 위에 그리드 표시
- 마우스 클릭으로 장애물 설정/제거
- 실제 좌표계로 변환
"""

import cv2
import numpy as np
from pupil_apriltags import Detector
import math
import heapq
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Set
from PySide6.QtWidgets import (QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout,
    QSpinBox,
    QListWidget,
    QListWidgetItem,
    QCheckBox,
    QGroupBox,
    QComboBox,
    QFrame,
    QScrollArea,
    QTabWidget,
    QProgressBar,
)
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QFont, QColor, QPalette
from PySide6.QtCore import Qt, Signal, QPoint, QTimer
import sys

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from typing import Dict, Optional, List
import time
import math
import json


# 로봇 ID와 AprilTag ID 매핑
ROBOT_CONFIG = {
    2: 4,
    3: 2,
}
@dataclass
class RobotStatus:
    """로봇 상태 정보를 담는 데이터 클래스"""
    robot_id: int
    
    # 위치 정보
    camera_pose: Optional[PoseStamped] = None
    target_pose: Optional[PoseStamped] = None
    
    # 제어 정보
    cmd_vel: Optional[Twist] = None
    
    # 시스템 정보
    battery: Optional[float] = None
    task: Optional[str] = None
    state: Optional[str] = None
    
    # 로봇팔 정보 (해당하는 경우)
    joint_states: Optional[JointState] = None
    
    # 타임스탬프
    last_camera_pose_time: float = 0.0
    last_target_pose_time: float = 0.0
    last_cmd_vel_time: float = 0.0
    last_battery_time: float = 0.0
    last_task_time: float = 0.0
    last_state_time: float = 0.0
    last_joint_states_time: float = 0.0
    
    def is_camera_pose_recent(self, timeout=2.0) -> bool:
        """카메라 포즈가 최근 데이터인지 확인"""
        return (time.time() - self.last_camera_pose_time) < timeout
    
    def is_target_pose_recent(self, timeout=5.0) -> bool:
        """타겟 포즈가 최근 데이터인지 확인"""
        return (time.time() - self.last_target_pose_time) < timeout
    
    def is_cmd_vel_recent(self, timeout=1.0) -> bool:
        """제어 명령이 최근 데이터인지 확인"""
        return (time.time() - self.last_cmd_vel_time) < timeout
    
    def is_battery_recent(self, timeout=10.0) -> bool:
        """배터리 데이터가 최근 데이터인지 확인"""
        return (time.time() - self.last_battery_time) < timeout
    
    def get_distance_to_target(self) -> Optional[float]:
        """현재 위치에서 목표까지의 거리 계산"""
        if not self.camera_pose or not self.target_pose:
            return None
        
        current_pos = self.camera_pose.pose.position
        target_pos = self.target_pose.pose.position
        
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        
        return math.sqrt(dx*dx + dy*dy)
    
    def get_current_speed(self) -> Optional[float]:
        """현재 속도 반환"""
        if not self.cmd_vel:
            return None
        
        linear_speed = math.sqrt(
            self.cmd_vel.linear.x**2 + 
            self.cmd_vel.linear.y**2
        )
        return linear_speed
    
    def to_dict(self) -> Dict:
        """딕셔너리 형태로 변환 (JSON 직렬화용)"""
        result = {
            'robot_id': self.robot_id,
            'task': self.task,
            'state': self.state,
            'battery': self.battery,
            'timestamps': {
                'camera_pose': self.last_camera_pose_time,
                'target_pose': self.last_target_pose_time,
                'cmd_vel': self.last_cmd_vel_time,
                'battery': self.last_battery_time,
                'task': self.last_task_time,
                'state': self.last_state_time
            }
        }
        
        # 위치 정보 추가
        if self.camera_pose:
            pos = self.camera_pose.pose.position
            result['current_position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
        
        if self.target_pose:
            pos = self.target_pose.pose.position
            result['target_position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
        
        # 속도 정보 추가
        if self.cmd_vel:
            result['velocity'] = {
                'linear': {'x': self.cmd_vel.linear.x, 'y': self.cmd_vel.linear.y},
                'angular': {'z': self.cmd_vel.angular.z}
            }
        
        # 계산된 정보 추가
        distance = self.get_distance_to_target()
        if distance is not None:
            result['distance_to_target'] = distance
        
        speed = self.get_current_speed()
        if speed is not None:
            result['current_speed'] = speed
        
        return result

class RobotStatusNode(Node):
    """로봇 위치 정보를 ROS2 토픽으로 발행하는 노드"""

    def __init__(self):
        super().__init__("robot_pub_sub")

        # 로봇 상태 저장소 초기화
        self.robot_statuses = {}
        for robot_id in ROBOT_CONFIG.keys():
            self.robot_statuses[robot_id] = RobotStatus(robot_id=robot_id)

        # 각 로봇에 대한 퍼블리셔 생성
        self.pose_publishers = {}
        self.target_pose_publishers = {}
        self.cmd_vel_publishers = {}

        for robot_id in ROBOT_CONFIG.keys():
            # 카메라 포즈 퍼블리셔 (현재 위치)
            pose_topic = f"/robot{robot_id}/camera_pose"
            pose_pub = self.create_publisher(
                PoseStamped, pose_topic, self.camera_pose_callback, 10
            )
            self.pose_publishers[robot_id] = pose_pub
            self.get_logger().info(f"Created pose publisher for {pose_topic}")

            # 목표 포즈 퍼블리셔 (웨이포인트)
            target_topic = f"/robot{robot_id}/target_pose"
            target_pub = self.create_publisher(PoseStamped, target_topic, 10)
            self.target_pose_publishers[robot_id] = target_pub
            self.get_logger().info(f"Created target pose publisher for {target_topic}")

            # 속도 제어 퍼블리셔
            cmd_vel_topic = f"/robot{robot_id}/cmd_vel"
            cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.cmd_vel_publishers[robot_id] = cmd_vel_pub
            self.get_logger().info(f"Created cmd_vel publisher for {cmd_vel_topic}")


            # camera 구독
            camera_sub = self.create_subscription(
                PoseStamped,
                f"/robot{robot_id}/camera_pose",
                lambda msg, rid=robot_id: self.camera_pose_callback(msg, rid),
                10,
            )

            # target 구독
            target_sub = self.create_subscription(
                PoseStamped,
                f"/robot{robot_id}/target_pose",
                lambda msg, rid=robot_id: self.target_pose_callback(msg, rid),
                10,
            )
            # cmd_vel 구독
            cmd_vel_sub = self.create_subscription(
                Twist,
                f"/robot{robot_id}/cmd_vel",
                lambda msg, rid=robot_id: self.cmd_vel_callback(msg, rid),
                10,
            )

            # Battery 구독
            battery_sub = self.create_subscription(
                Float32,
                f"/robot{robot_id}/battery",
                lambda msg, rid=robot_id: self.battery_callback(msg, rid),
                10,
            )

            # Task 구독 (String 타입으로 가정)
            task_sub = self.create_subscription(
                String,
                f"/robot{robot_id}/task",
                lambda msg, rid=robot_id: self.task_callback(msg, rid),
                10,
            )

            # State 구독 (String 타입으로 가정)
            state_sub = self.create_subscription(
                String,
                f"/robot{robot_id}/state",
                lambda msg, rid=robot_id: self.state_callback(msg, rid),
                10,
            )

            # Joint States 구독 (로봇팔용)
            joint_states_sub = self.create_subscription(
                JointState,
                f"/robot{robot_id}/joint_states",
                lambda msg, rid=robot_id: self.joint_states_callback(msg, rid),
                10,
            )

    def camera_pose_callback(self, msg: PoseStamped, robot_id: int):
        """카메라 포즈 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].camera_pose = msg
            self.robot_statuses[robot_id].last_camera_pose_time = time.time()

    def target_pose_callback(self, msg: PoseStamped, robot_id: int):
        """타겟 포즈 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].target_pose = msg
            self.robot_statuses[robot_id].last_target_pose_time = time.time()

            pos = msg.pose.position
            self.get_logger().debug(
                f"🎯 Robot {robot_id} target updated: ({pos.x:.2f}, {pos.y:.2f})"
            )

    def cmd_vel_callback(self, msg: Twist, robot_id: int):
        """속도 명령 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].cmd_vel = msg
            self.robot_statuses[robot_id].last_cmd_vel_time = time.time()

    def battery_callback(self, msg: Float32, robot_id: int):
        """배터리 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].battery = msg.data
            self.robot_statuses[robot_id].last_battery_time = time.time()

            # 배터리 부족 경고
            if msg.data < 0.2:  # 20% 미만
                self.get_logger().warn(
                    f"🔋 Robot {robot_id} LOW BATTERY: {msg.data*100:.1f}%"
                )

    def task_callback(self, msg: String, robot_id: int):
        """작업 상태 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].task = msg.data
            self.robot_statuses[robot_id].last_task_time = time.time()

            self.get_logger().info(f"📋 Robot {robot_id} task: {msg.data}")

    def state_callback(self, msg: String, robot_id: int):
        """로봇 상태 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].state = msg.data
            self.robot_statuses[robot_id].last_state_time = time.time()

            self.get_logger().info(f"🤖 Robot {robot_id} state: {msg.data}")

    def joint_states_callback(self, msg: JointState, robot_id: int):
        """관절 상태 콜백 (로봇팔용)"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].joint_states = msg
            self.robot_statuses[robot_id].last_joint_states_time = time.time()

            self.get_logger().info(f"📡 Subscribed to all topics for robot {robot_id}")

    def get_robot_statuses(self):
        """모든 로봇 상태를 딕셔너리 형태로 반환"""
        statuses_dict = {}
        for robot_id, status in self.robot_statuses.items():
            statuses_dict[robot_id] = status.to_dict()
        return statuses_dict

    def get_robot_status(self, robot_id: int):
        """특정 로봇 상태 반환"""
        return self.robot_statuses.get(robot_id)

    def is_robot_online(self, robot_id: int) -> bool:
        """로봇이 온라인 상태인지 확인"""
        if robot_id not in self.robot_statuses:
            return False
        status = self.robot_statuses[robot_id]
        return (status.is_camera_pose_recent() or status.is_battery_recent())

    def publish_robot_pose(self, robot_id, x, y, yaw):
        """로봇 위치를 PoseStamped 메시지로 발행"""
        if robot_id not in self.pose_publishers:
            return

        # PoseStamped 메시지 생성
        pose_msg = PoseStamped()

        # Header 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_frame"

        # Position 설정
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0

        # Orientation 설정 (yaw를 quaternion으로 변환)
        quat = self.euler_to_quaternion(0, 0, yaw)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # 메시지 발행
        self.pose_publishers[robot_id].publish(pose_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각을 쿼터니언으로 변환"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def publish_target_pose(self, robot_id, target_x, target_y, target_yaw=0.0):
        """목표 위치(웨이포인트)를 ROS2 토픽으로 발행"""
        if robot_id not in self.target_pose_publishers:
            return

        # PoseStamped 메시지 생성
        pose_msg = PoseStamped()

        # Header 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Position 설정
        pose_msg.pose.position.x = float(target_x)
        pose_msg.pose.position.y = float(target_y)
        pose_msg.pose.position.z = 0.0

        # Orientation 설정
        quat = self.euler_to_quaternion(0, 0, target_yaw)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # 메시지 발행
        self.target_pose_publishers[robot_id].publish(pose_msg)
        self.get_logger().info(
            f"Published target pose for robot{robot_id}: ({target_x:.2f}, {target_y:.2f})"
        )

    def publish_cmd_vel(self, robot_id, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """로봇 속도 제어 명령 발행"""
        if robot_id not in self.cmd_vel_publishers:
            return

        # Twist 메시지 생성
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = float(linear_y)
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(angular_z)

        # 메시지 발행
        self.cmd_vel_publishers[robot_id].publish(twist_msg)

    def stop_robot(self, robot_id):
        """로봇 정지"""
        self.publish_cmd_vel(robot_id, 0.0, 0.0, 0.0)


@dataclass
class PathNode:
    """경로 계획용 노드"""

    row: int
    col: int
    g_cost: float = 0.0  # 시작점부터의 비용
    h_cost: float = 0.0  # 목표점까지의 추정 비용
    parent: Optional["PathNode"] = None

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        if isinstance(other, PathNode):
            return self.row == other.row and self.col == other.col
        return False

    def __hash__(self):
        return hash((self.row, self.col))


class MultiRobotPathPlanner:
    """멀티 로봇 경로 계획 시스템"""

    def __init__(self, grid_rows: int, grid_cols: int):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.obstacles: Set[Tuple[int, int]] = set()
        self.robot_paths: Dict[int, List[Tuple[int, int]]] = {}
        self.robot_goals: Dict[int, Tuple[int, int]] = {}
        self.safety_margin = 1  # 로봇 간 안전 거리 (그리드 셀 단위)

    def set_obstacles(self, obstacles: Set[Tuple[int, int]]):
        """장애물 위치 설정"""
        self.obstacles = obstacles.copy()

    def set_robot_goal(self, robot_id: int, goal: Tuple[int, int]):
        """로봇 목표 설정"""
        self.robot_goals[robot_id] = goal

    def is_valid_position(self, row: int, col: int, exclude_robot: int = None) -> bool:
        """유효한 위치인지 확인"""
        # 그리드 범위 확인
        if not (0 <= row < self.grid_rows and 0 <= col < self.grid_cols):
            return False

        # 장애물 확인
        if (row, col) in self.obstacles:
            return False

        # 다른 로봇과의 충돌 확인
        for robot_id, path in self.robot_paths.items():
            if robot_id == exclude_robot:
                continue
            if path and len(path) > 0:
                # 현재 로봇 위치와 안전 거리 확인
                robot_pos = path[0] if path else None
                if (
                    robot_pos
                    and self.manhattan_distance((row, col), robot_pos)
                    <= self.safety_margin
                ):
                    return False

        return True

    def manhattan_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> int:
        """맨해튼 거리 계산"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def euclidean_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """유클리드 거리 계산"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def get_neighbors(
        self, node: PathNode, exclude_robot: int = None
    ) -> List[PathNode]:
        """인접 노드 가져오기 (8방향)"""
        neighbors = []
        directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        for dr, dc in directions:
            new_row, new_col = node.row + dr, node.col + dc
            if self.is_valid_position(new_row, new_col, exclude_robot):
                neighbors.append(PathNode(new_row, new_col))

        return neighbors

    def astar_pathfind(
        self, start: Tuple[int, int], goal: Tuple[int, int], exclude_robot: int = None
    ) -> List[Tuple[int, int]]:
        """A* 알고리즘으로 경로 탐색"""
        start_node = PathNode(start[0], start[1])
        goal_node = PathNode(goal[0], goal[1])

        open_list = [start_node]
        closed_set = set()

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node == goal_node:
                # 경로 재구성
                path = []
                while current_node:
                    path.append((current_node.row, current_node.col))
                    current_node = current_node.parent
                return path[::-1]

            closed_set.add((current_node.row, current_node.col))

            for neighbor in self.get_neighbors(current_node, exclude_robot):
                if (neighbor.row, neighbor.col) in closed_set:
                    continue

                # 대각선 이동인지 확인
                is_diagonal = (
                    abs(neighbor.row - current_node.row) == 1
                    and abs(neighbor.col - current_node.col) == 1
                )
                move_cost = (
                    1.414 if is_diagonal else 1.0
                )  # 대각선 이동은 더 비용이 높음

                tentative_g_cost = current_node.g_cost + move_cost

                # 기존 노드보다 더 나은 경로인지 확인
                existing_node = None
                for node in open_list:
                    if node == neighbor:
                        existing_node = node
                        break

                if existing_node is None or tentative_g_cost < existing_node.g_cost:
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self.euclidean_distance(
                        (neighbor.row, neighbor.col), goal
                    )
                    neighbor.parent = current_node

                    if existing_node is None:
                        heapq.heappush(open_list, neighbor)

        return []  # 경로를 찾을 수 없음

    def plan_multi_robot_paths(
        self, robot_positions: Dict[int, Tuple[int, int]]
    ) -> Dict[int, List[Tuple[int, int]]]:
        """멀티 로봇 경로 계획"""
        planned_paths = {}

        # 우선순위: 목표가 더 가까운 로봇부터 계획
        robot_priorities = []
        for robot_id, position in robot_positions.items():
            if robot_id in self.robot_goals:
                goal = self.robot_goals[robot_id]
                distance = self.euclidean_distance(position, goal)
                robot_priorities.append((distance, robot_id, position))

        robot_priorities.sort()  # 거리 순으로 정렬

        # 각 로봇에 대해 순차적으로 경로 계획
        for _, robot_id, position in robot_priorities:
            if robot_id in self.robot_goals:
                goal = self.robot_goals[robot_id]
                path = self.astar_pathfind(position, goal, exclude_robot=robot_id)
                if path:
                    planned_paths[robot_id] = path
                    self.robot_paths[robot_id] = path
                else:
                    # 경로를 찾을 수 없으면 현재 위치 유지
                    planned_paths[robot_id] = [position]
                    self.robot_paths[robot_id] = [position]

        return planned_paths


class GridCameraApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🤖 로봇 플릿 모니터링 & 제어 시스템")
        self.setGeometry(100, 100, 1024, 600)

        # ROS2 초기화 (UI 초기화 전에 수행)
        self.ros_node = None
        self.status_subscriber = None
        self.ros_status = None  # UI 생성 전 임시 설정

        # === 설정 값 ===
        self.camera_index = 2
        self.grid_rows = 10
        self.grid_cols = 15
        self.real_width = 2  # 실제 너비 (미터)
        self.real_height = 1  # 실제 높이 (미터)

        # === 내부 변수 ===
        self.camera = None
        self.current_frame = None
        self.grid_corners = None  # [top_left, top_right, bottom_left, bottom_right]
        self.obstacles = set()  # 장애물 위치 저장 (grid_row, grid_col)
        self.corner_points = []  # 사용자가 클릭한 4개 코너 점
        self.is_setting_corners = True

        # === AprilTag 관련 ===
        self.apriltag_detector = Detector(families="tag36h11")
        self.enable_apriltag = False
        self.apriltag_detections = []
        self.robot_positions = {}  # 로봇 위치 저장

        # === 경로 계획 관련 ===
        self.path_planner = MultiRobotPathPlanner(self.grid_rows, self.grid_cols)
        self.robot_goals = {}  # 로봇 목표 위치
        self.robot_paths = {}  # 로봇 경로
        self.setting_goal_for_robot = None  # 목표 설정 모드
        self.enable_path_planning = False

        # === 웨이포인트 관련 ===
        self.robot_waypoints = {}  # 로봇별 웨이포인트 리스트 {robot_id: [(x,y), ...]}
        self.robot_current_waypoint_index = {}  # 로봇별 현재 웨이포인트 인덱스
        self.waypoint_tolerance = 0.1  # 웨이포인트 도달 허용 오차 (미터)
        self.enable_robot_control = False  # 로봇 제어 활성화 여부
        self.robot_target_published = {}  # 로봇별 목표 발행 상태

        self.init_ui()
        self.init_camera()

        # ROS2 초기화 (UI 생성 후 수행)
        self.init_ros2()

        # 타이머로 프레임 업데이트
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30ms마다 업데이트 (약 33 FPS)

        # ROS2 스핀 타이머
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 10ms마다 ROS2 스핀

        # 로봇 상태 업데이트 타이머
        self.robot_status_timer = QTimer()
        self.robot_status_timer.timeout.connect(self.update_robot_monitor_tab)
        self.robot_status_timer.start(500)  # 0.5초마다 업데이트

    def init_ui(self):
        """UI 초기화"""
        main_layout = QVBoxLayout()

        # 탭 위젯 생성
        tabs = QTabWidget()

        # 카메라 제어 탭
        camera_tab = QWidget()
        self.setup_camera_tab(camera_tab)
        tabs.addTab(camera_tab, "📷 카메라 제어")

        # 로봇 모니터 탭
        monitor_tab = QWidget()
        self.setup_robot_monitor_tab(monitor_tab)
        tabs.addTab(monitor_tab, "🤖 로봇 모니터")

        main_layout.addWidget(tabs)
        self.setLayout(main_layout)

    def setup_camera_tab(self, tab):
        """카메라 제어 탭 UI 설정"""
        # === 메인 레이아웃 ===
        main_layout = QVBoxLayout()

        # === 중앙: 컨트롤 패널과 카메라 영상 ===
        center_layout = QHBoxLayout()

        # 왼쪽 컨트롤 패널
        left_panel = QVBoxLayout()

        # 그리드 크기 설정
        grid_group = QGroupBox("⚙️ 그리드 설정")
        grid_layout = QGridLayout()
        grid_layout.addWidget(QLabel("행 수:"), 0, 0)
        self.rows_spinbox = QSpinBox()
        self.rows_spinbox.setRange(5, 20)
        self.rows_spinbox.setValue(self.grid_rows)
        self.rows_spinbox.valueChanged.connect(self.update_grid_size)
        grid_layout.addWidget(self.rows_spinbox, 0, 1)

        grid_layout.addWidget(QLabel("열 수:"), 1, 0)
        self.cols_spinbox = QSpinBox()
        self.cols_spinbox.setRange(5, 30)
        self.cols_spinbox.setValue(self.grid_cols)
        self.cols_spinbox.valueChanged.connect(self.update_grid_size)
        grid_layout.addWidget(self.cols_spinbox, 1, 1)
        grid_group.setLayout(grid_layout)
        left_panel.addWidget(grid_group)

        # 카메라 설정
        camera_group = QGroupBox("📷 카메라 설정")
        camera_layout = QVBoxLayout()

        # 코너 설정 버튼
        self.corner_button = QPushButton("🎯 코너 4개 점 설정하기")
        self.corner_button.clicked.connect(self.start_corner_setting)
        camera_layout.addWidget(self.corner_button)

        # 상태 표시
        self.status_label = QLabel("카메라 영상의 4개 코너를 순서대로 클릭하세요")
        self.status_label.setWordWrap(True)
        camera_layout.addWidget(self.status_label)
        camera_group.setLayout(camera_layout)
        left_panel.addWidget(camera_group)

        # 장애물 및 웨이포인트 리스트
        lists_group = QGroupBox("📋 목록")
        lists_layout = QVBoxLayout()

        # 장애물 리스트
        lists_layout.addWidget(QLabel("🚧 장애물:"))
        self.obstacle_list = QListWidget()
        self.obstacle_list.setMaximumHeight(100)
        lists_layout.addWidget(self.obstacle_list)

        # 웨이포인트 리스트
        lists_layout.addWidget(QLabel("🎯 웨이포인트:"))
        self.waypoint_list = QListWidget()
        self.waypoint_list.setMaximumHeight(100)
        lists_layout.addWidget(self.waypoint_list)

        lists_group.setLayout(lists_layout)
        left_panel.addWidget(lists_group)

        # AprilTag 설정
        apriltag_group = QGroupBox("🏷️ AprilTag 감지")
        apriltag_layout = QVBoxLayout()

        self.apriltag_checkbox = QCheckBox("AprilTag 감지 활성화")
        self.apriltag_checkbox.stateChanged.connect(self.toggle_apriltag)
        apriltag_layout.addWidget(self.apriltag_checkbox)

        self.apriltag_status = QLabel("AprilTag: 비활성화")
        apriltag_layout.addWidget(self.apriltag_status)

        apriltag_group.setLayout(apriltag_layout)
        left_panel.addWidget(apriltag_group)

        # ROS2 상태 표시
        ros_group = QGroupBox("📡 ROS2 상태")
        ros_layout = QVBoxLayout()

        self.ros_status = QLabel("ROS2: 초기화 중...")
        ros_layout.addWidget(self.ros_status)

        ros_group.setLayout(ros_layout)
        left_panel.addWidget(ros_group)

        # 경로 계획 설정
        path_group = QGroupBox("🛣️ 경로 계획")
        path_layout = QVBoxLayout()

        self.path_planning_checkbox = QCheckBox("경로 계획 활성화")
        self.path_planning_checkbox.stateChanged.connect(self.toggle_path_planning)
        path_layout.addWidget(self.path_planning_checkbox)

        # 로봇 목표 설정
        goal_layout = QHBoxLayout()
        goal_layout.addWidget(QLabel("로봇 목표:"))
        self.robot_goal_combo = QComboBox()
        for robot_id in ROBOT_CONFIG.keys():
            self.robot_goal_combo.addItem(f"로봇 {robot_id}")
        goal_layout.addWidget(self.robot_goal_combo)

        self.set_goal_button = QPushButton("목표 설정")
        self.set_goal_button.clicked.connect(self.start_goal_setting)
        goal_layout.addWidget(self.set_goal_button)
        path_layout.addLayout(goal_layout)

        self.plan_path_button = QPushButton("경로 계획 실행")
        self.plan_path_button.clicked.connect(self.plan_robot_paths)
        path_layout.addWidget(self.plan_path_button)

        self.clear_paths_button = QPushButton("경로 지우기")
        self.clear_paths_button.clicked.connect(self.clear_all_paths)
        path_layout.addWidget(self.clear_paths_button)

        # 웨이포인트 관련 UI
        waypoint_layout = QHBoxLayout()
        self.test_waypoints_button = QPushButton("테스트 웨이포인트 설정")
        self.test_waypoints_button.clicked.connect(self.set_test_waypoints)
        waypoint_layout.addWidget(self.test_waypoints_button)

        self.clear_waypoints_button = QPushButton("웨이포인트 지우기")
        self.clear_waypoints_button.clicked.connect(self.clear_waypoints)
        waypoint_layout.addWidget(self.clear_waypoints_button)
        path_layout.addLayout(waypoint_layout)

        # 로봇 제어 관련 UI
        control_layout = QVBoxLayout()

        self.robot_control_checkbox = QCheckBox("로봇 자동 제어 활성화")
        self.robot_control_checkbox.stateChanged.connect(self.toggle_robot_control)
        control_layout.addWidget(self.robot_control_checkbox)

        control_buttons_layout = QHBoxLayout()
        self.start_waypoint_mission_button = QPushButton("웨이포인트 미션 시작")
        self.start_waypoint_mission_button.clicked.connect(self.start_waypoint_mission)
        control_buttons_layout.addWidget(self.start_waypoint_mission_button)

        self.stop_all_robots_button = QPushButton("모든 로봇 정지")
        self.stop_all_robots_button.clicked.connect(self.stop_all_robots)
        control_buttons_layout.addWidget(self.stop_all_robots_button)
        control_layout.addLayout(control_buttons_layout)

        path_layout.addLayout(control_layout)

        path_group.setLayout(path_layout)
        left_panel.addWidget(path_group)

        # 초기화 버튼
        clear_button = QPushButton("🗑️ 모든 장애물 제거")
        clear_button.clicked.connect(self.clear_obstacles)
        left_panel.addWidget(clear_button)

        left_panel.addStretch()

        # === 오른쪽: 카메라 영상 ===
        self.image_label = ClickableLabel()
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setStyleSheet("border: 1px solid gray")
        self.image_label.mouse_clicked.connect(self.handle_mouse_click)

        # 레이아웃 조합
        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(300)

        center_layout.addWidget(left_widget)
        center_layout.addWidget(self.image_label, 1)

        main_layout.addLayout(center_layout)
        tab.setLayout(main_layout)

    def create_robot_status_panel(self):
        """로봇 상태 패널 생성"""
        # 메인 프레임
        status_frame = QFrame()
        status_frame.setFrameStyle(QFrame.Shape.StyledPanel)
        status_frame.setStyleSheet(
            """
            QFrame {
                background-color: #f0f0f0;
                border: 2px solid #cccccc;
                border-radius: 8px;
                margin: 5px;
                padding: 10px;
            }
        """
        )

        # 레이아웃
        status_layout = QVBoxLayout()

        # 타이틀
        title_layout = QHBoxLayout()
        title_label = QLabel("🤖 로봇 상태 모니터링")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(14)
        title_label.setFont(title_font)
        title_layout.addWidget(title_label)

        # 온라인 로봇 수 표시
        self.online_count_label = QLabel("온라인: 0대")
        self.online_count_label.setStyleSheet("color: #008000; font-weight: bold;")
        title_layout.addWidget(self.online_count_label)

        title_layout.addStretch()
        status_layout.addLayout(title_layout)

        # 로봇 정보 카드들을 담을 수평 스크롤 영역
        scroll_area = QScrollArea()
        scroll_area.setFixedHeight(120)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        # 로봇 카드 컨테이너
        self.robot_cards_widget = QWidget()
        self.robot_cards_layout = QHBoxLayout()
        self.robot_cards_layout.setSpacing(10)
        self.robot_cards_widget.setLayout(self.robot_cards_layout)

        scroll_area.setWidget(self.robot_cards_widget)
        status_layout.addWidget(scroll_area)

        status_frame.setLayout(status_layout)
        status_frame.setFixedHeight(180)

        return status_frame

    def setup_robot_monitor_tab(self, tab):
        """로봇 모니터 탭 UI 설정"""
        layout = QVBoxLayout()

        title = QLabel("🤖 실시간 로봇 상태 모니터")
        font = title.font()
        font.setPointSize(16)
        font.setBold(True)
        title.setFont(font)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.robot_monitor_layout = QGridLayout()
        layout.addLayout(self.robot_monitor_layout)

        layout.addStretch()
        tab.setLayout(layout)

    def update_robot_monitor_tab(self):
        """로봇 모니터 탭 정보 업데이트"""
        if not self.status_subscriber:
            return

        # 기존 위젯 제거
        for i in reversed(range(self.robot_monitor_layout.count())):
            self.robot_monitor_layout.itemAt(i).widget().setParent(None)

        statuses = self.status_subscriber.robot_statuses

        row, col = 0, 0
        for robot_id, status in statuses.items():
            card = self.create_robot_monitor_card(status)
            self.robot_monitor_layout.addWidget(card, row, col)
            col += 1
            if col > 2:
                col = 0
                row += 1

    def create_robot_monitor_card(self, status: RobotStatus):
        """로봇 모니터링용 카드 생성"""
        card = QGroupBox(f"🤖 Robot {status.robot_id}")
        layout = QGridLayout()

        # 상태
        state_label = QLabel(f"<b>상태:</b> {status.state or 'N/A'}")
        layout.addWidget(state_label, 0, 0, 1, 2)

        # 배터리
        battery_label = QLabel("<b>배터리:</b>")
        battery_bar = QProgressBar()
        battery_bar.setRange(0, 100)
        battery_bar.setValue(int((status.battery or 0) * 100))
        battery_bar.setTextVisible(True)
        battery_bar.setFormat(f"{int((status.battery or 0) * 100)}%")
        layout.addWidget(battery_label, 1, 0)
        layout.addWidget(battery_bar, 1, 1)

        # 위치
        pos_text = "N/A"
        if status.camera_pose:
            pos = status.camera_pose.pose.position
            pos_text = f"({pos.x:.2f}, {pos.y:.2f})"
        pos_label = QLabel(f"<b>위치:</b> {pos_text}")
        layout.addWidget(pos_label, 2, 0, 1, 2)

        # 목표
        target_text = "N/A"
        if status.target_pose:
            target = status.target_pose.pose.position
            target_text = f"({target.x:.2f}, {target.y:.2f})"
        target_label = QLabel(f"<b>목표:</b> {target_text}")
        layout.addWidget(target_label, 3, 0, 1, 2)

        # 작업
        task_label = QLabel(f"<b>작업:</b> {status.task or 'N/A'}")
        layout.addWidget(task_label, 4, 0, 1, 2)

        card.setLayout(layout)
        return card

    def init_camera(self):
        """카메라 초기화"""
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            self.status_label.setText("❌ 카메라를 열 수 없습니다")
            return False

        # 해상도 설정
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return True

    def update_frame(self):
        """프레임 업데이트"""
        if not self.camera or not self.camera.isOpened():
            return

        ret, frame = self.camera.read()
        if not ret:
            return

        self.current_frame = frame.copy()

        # AprilTag 감지
        if self.enable_apriltag:
            self.detect_apriltags()

        # 그리드와 장애물 그리기
        self.draw_grid_and_obstacles()

        # Qt용 이미지로 변환
        self.display_frame()

    def detect_apriltags(self):
        """AprilTag 감지 및 로봇 위치 추적"""
        if self.current_frame is None:
            return

        gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
        detections = self.apriltag_detector.detect(gray)

        self.apriltag_detections = []
        self.robot_positions = {}  # 로봇 위치 초기화

        for detection in detections:
            tag_id = detection.tag_id
            center = detection.center.astype(int)
            corners = detection.corners.astype(int)

            # 태그 정보 저장
            self.apriltag_detections.append(
                {"tag_id": tag_id, "center": center, "corners": corners}
            )

            # 로봇 추적 (로봇 태그인지 확인)
            robot_id = self.get_robot_id_by_tag(tag_id)
            if robot_id is not None:
                # 그리드 좌표 계산
                grid_pos = self.pos_to_grid(tuple(center))
                real_coords = self.image_to_real_coords(center[0], center[1])

                # yaw 각도 계산 (x축 기준 라디안)
                yaw_radians = self.calculate_yaw_from_corners(corners)

                self.robot_positions[robot_id] = {
                    "tag_id": tag_id,
                    "center": center,
                    "corners": corners,
                    "grid_pos": grid_pos,
                    "real_coords": real_coords,
                    "yaw": yaw_radians,
                }

        # 상태 업데이트
        robot_count = len(self.robot_positions)
        total_tags = len(self.apriltag_detections)
        if total_tags > 0:
            self.apriltag_status.setText(
                f"AprilTag: {total_tags}개 감지 (로봇: {robot_count}개)"
            )
        else:
            self.apriltag_status.setText("AprilTag: 감지되지 않음")

        # 로봇 위치 리스트 업데이트
        self.update_robot_list()

        # 콘솔에 로봇 위치 출력
        self.print_robot_positions()

    def draw_grid_and_obstacles(self):
        """그리드와 장애물을 프레임에 그리기"""
        if self.current_frame is None:
            return

        # AprilTag 그리기
        if self.enable_apriltag:
            self.draw_apriltags()

        # 코너 점들 그리기
        for i, point in enumerate(self.corner_points):
            color = (0, 255, 0) if i < 4 else (0, 0, 255)
            cv2.circle(self.current_frame, point, 8, color, -1)
            cv2.putText(
                self.current_frame,
                str(i + 1),
                (point[0] + 10, point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )

        # 그리드 그리기 (4개 코너가 설정된 경우)
        if len(self.corner_points) >= 4:
            self.draw_grid()
            self.draw_obstacles()
            self.draw_robot_goals()
            self.draw_robot_paths()
            self.draw_waypoints()

    def draw_grid(self):
        """그리드 선 그리기"""
        corners = np.array(self.corner_points[:4], dtype=np.float32)

        # 각 그리드 포인트 계산
        for row in range(self.grid_rows + 1):
            for col in range(self.grid_cols + 1):
                # 비율 계산
                row_ratio = row / self.grid_rows
                col_ratio = col / self.grid_cols

                # 양선형 보간으로 그리드 포인트 계산
                top_point = corners[0] + col_ratio * (corners[1] - corners[0])
                bottom_point = corners[2] + col_ratio * (corners[3] - corners[2])
                grid_point = top_point + row_ratio * (bottom_point - top_point)

                # 세로선 그리기
                if col < self.grid_cols:
                    next_top = corners[0] + (col_ratio + 1 / self.grid_cols) * (
                        corners[1] - corners[0]
                    )
                    next_bottom = corners[2] + (col_ratio + 1 / self.grid_cols) * (
                        corners[3] - corners[2]
                    )
                    next_grid = next_top + row_ratio * (next_bottom - next_top)
                    cv2.line(
                        self.current_frame,
                        tuple(grid_point.astype(int)),
                        tuple(next_grid.astype(int)),
                        (255, 255, 0),
                        1,
                    )

                # 가로선 그리기
                if row < self.grid_rows:
                    next_row_ratio = (row + 1) / self.grid_rows
                    next_top_point = corners[0] + col_ratio * (corners[1] - corners[0])
                    next_bottom_point = corners[2] + col_ratio * (
                        corners[3] - corners[2]
                    )
                    next_grid_point = next_top_point + next_row_ratio * (
                        next_bottom_point - next_top_point
                    )
                    cv2.line(
                        self.current_frame,
                        tuple(grid_point.astype(int)),
                        tuple(next_grid_point.astype(int)),
                        (255, 255, 0),
                        1,
                    )

    def draw_obstacles(self):
        """장애물 그리기"""
        if len(self.corner_points) < 4:
            return

        corners = np.array(self.corner_points[:4], dtype=np.float32)

        for row, col in self.obstacles:
            # 격자 셀의 4개 모서리 계산
            cell_corners = []
            for r_offset, c_offset in [(0, 0), (0, 1), (1, 1), (1, 0)]:
                cell_row = (row + r_offset) / self.grid_rows
                cell_col = (col + c_offset) / self.grid_cols

                top_point = corners[0] + cell_col * (corners[1] - corners[0])
                bottom_point = corners[2] + cell_col * (corners[3] - corners[2])
                cell_point = top_point + cell_row * (bottom_point - top_point)
                cell_corners.append(cell_point.astype(int))

            # 장애물을 빨간색으로 채우기
            pts = np.array(cell_corners)
            cv2.fillPoly(self.current_frame, [pts], (0, 0, 255))
            cv2.polylines(self.current_frame, [pts], True, (0, 0, 128), 2)

    def display_frame(self):
        """프레임을 Qt 위젯에 표시"""
        if self.current_frame is None:
            return

        # BGR to RGB
        rgb_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w

        q_image = QImage(
            rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
        )
        pixmap = QPixmap.fromImage(q_image)

        # 크기에 맞게 조정
        scaled_pixmap = pixmap.scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )

        self.image_label.setPixmap(scaled_pixmap)

    def handle_mouse_click(self, pos):
        """마우스 클릭 처리"""
        if self.current_frame is None:
            return

        # Qt 좌표를 실제 이미지 좌표로 변환
        real_pos = self.qt_to_image_coords(pos)
        if real_pos is None:
            return

        if self.is_setting_corners:
            self.add_corner_point(real_pos)
        elif self.setting_goal_for_robot is not None:
            self.set_robot_goal(real_pos)
        else:
            self.toggle_obstacle(real_pos)

    def qt_to_image_coords(self, qt_pos):
        """Qt 좌표를 실제 이미지 좌표로 변환"""
        if self.current_frame is None:
            return None

        label_size = self.image_label.size()
        pixmap = self.image_label.pixmap()
        if not pixmap:
            return None

        # 스케일 계산
        img_h, img_w = self.current_frame.shape[:2]
        pixmap_size = pixmap.size()

        scale_x = pixmap_size.width() / img_w
        scale_y = pixmap_size.height() / img_h
        scale = min(scale_x, scale_y)

        # 실제 표시된 이미지 크기
        display_w = int(img_w * scale)
        display_h = int(img_h * scale)

        # 중앙 정렬 오프셋
        offset_x = (label_size.width() - display_w) // 2
        offset_y = (label_size.height() - display_h) // 2

        # 좌표 변환
        x = int((qt_pos.x() - offset_x) / scale)
        y = int((qt_pos.y() - offset_y) / scale)

        if 0 <= x < img_w and 0 <= y < img_h:
            return (x, y)
        return None

    def add_corner_point(self, pos):
        """코너 점 추가"""
        self.corner_points.append(pos)

        if len(self.corner_points) == 4:
            self.is_setting_corners = False
            self.status_label.setText(
                "✅ 코너 설정 완료! 이제 그리드를 클릭해서 장애물을 설정하세요"
            )
            self.corner_button.setText("🔄 코너 다시 설정")
        else:
            self.status_label.setText(f"코너 {len(self.corner_points)}/4 설정됨")

    def toggle_obstacle(self, pos):
        """장애물 토글 (추가/제거)"""
        if len(self.corner_points) < 4:
            return

        # 클릭 위치가 어느 그리드 셀에 속하는지 계산
        grid_pos = self.pos_to_grid(pos)
        if grid_pos is None:
            return

        row, col = grid_pos
        if (row, col) in self.obstacles:
            self.obstacles.remove((row, col))
        else:
            self.obstacles.add((row, col))

        self.update_obstacle_list()

        # 실제 좌표 출력
        real_coords = self.grid_to_real_coords(row, col)
        if real_coords:
            real_x, real_y = real_coords
            print(f"그리드 ({row}, {col}) → 실제 좌표 ({real_x:.2f}, {real_y:.2f})")

    def set_robot_goal(self, pos):
        """로봇 목표 설정"""
        if len(self.corner_points) < 4 or self.setting_goal_for_robot is None:
            return

        grid_pos = self.pos_to_grid(pos)
        if grid_pos is None:
            return

        row, col = grid_pos
        robot_id = self.setting_goal_for_robot

        # 목표 설정
        self.robot_goals[robot_id] = (row, col)
        self.path_planner.set_robot_goal(robot_id, (row, col))

        # UI 업데이트
        self.setting_goal_for_robot = None
        self.set_goal_button.setText("목표 설정")

        real_coords = self.grid_to_real_coords(row, col)
        if real_coords:
            real_x, real_y = real_coords
            print(
                f"🎯 로봇{robot_id} 목표 설정: 그리드({row},{col}) 실제({real_x:.2f},{real_y:.2f})"
            )

    def pos_to_grid(self, pos):
        """이미지 좌표를 그리드 좌표로 변환"""
        if len(self.corner_points) < 4:
            return None

        corners = np.array(self.corner_points[:4], dtype=np.float32)

        # 투시 변환으로 그리드 좌표 계산
        unit_square = np.array(
            [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=np.float32
        )

        try:
            matrix = cv2.getPerspectiveTransform(corners, unit_square)
            point = np.array([[[float(pos[0]), float(pos[1])]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(point, matrix)

            u, v = transformed[0][0]
            row = int(v * self.grid_rows)
            col = int(u * self.grid_cols)

            if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
                return (row, col)
        except:
            pass

        return None

    def grid_to_real_coords(self, row, col):
        """그리드 좌표를 실제 좌표로 변환"""
        real_x = (col + 0.5) * self.real_width / self.grid_cols
        real_y = (row + 0.5) * self.real_height / self.grid_rows
        return (real_x, real_y)

    def update_obstacle_list(self):
        """장애물 리스트 UI 업데이트"""
        self.obstacle_list.clear()
        for row, col in sorted(self.obstacles):
            real_coords = self.grid_to_real_coords(row, col)
            if real_coords:
                real_x, real_y = real_coords
                item_text = f"그리드({row},{col}) → 실제({real_x:.2f},{real_y:.2f})"
                self.obstacle_list.addItem(item_text)

    def start_corner_setting(self):
        """코너 설정 시작"""
        self.corner_points.clear()
        self.obstacles.clear()
        self.is_setting_corners = True
        self.status_label.setText("카메라 영상의 4개 코너를 순서대로 클릭하세요")
        self.corner_button.setText("🎯 코너 4개 점 설정하기")
        self.update_obstacle_list()

    def update_grid_size(self):
        """그리드 크기 업데이트"""
        self.grid_rows = self.rows_spinbox.value()
        self.grid_cols = self.cols_spinbox.value()
        self.obstacles.clear()  # 크기 변경시 장애물 초기화
        self.update_obstacle_list()

        # 경로 계획자 업데이트
        self.path_planner = MultiRobotPathPlanner(self.grid_rows, self.grid_cols)
        self.robot_goals = {}
        self.robot_paths = {}
        self.robot_waypoints = {}
        self.robot_current_waypoint_index = {}
        self.robot_target_published = {}

    def get_robot_id_by_tag(self, tag_id):
        """태그 ID로 로봇 ID 찾기"""
        for robot_id, robot_tag_id in ROBOT_CONFIG.items():
            if robot_tag_id == tag_id:
                return robot_id
        return None

    def calculate_yaw_from_corners(self, corners):
        """코너 점들로부터 yaw 각도 계산 (x축 기준 라디안)"""
        # AprilTag 코너 순서: [bottom-left, bottom-right, top-right, top-left]
        # x축 방향을 얻기 위해 오른쪽 변의 방향 벡터 사용
        bottom_right = corners[1]
        top_right = corners[2]

        # 오른쪽 변의 방향 벡터 (위쪽 방향)
        direction_vector = top_right - bottom_right

        # x축 기준 각도 계산
        yaw = math.atan2(direction_vector[1], direction_vector[0])

        # 90도 보정 (오른쪽 변이 위쪽 방향이므로, 실제 로봇 전면 방향은 -90도 회전)
        yaw -= math.pi / 2

        # -π에서 π 범위로 정규화
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi

        return yaw

    def get_robot_color(self, robot_id):
        """로봇별 색상 반환"""
        colors = {
            1: (0, 255, 0),  # 초록색
            2: (255, 0, 0),  # 빨간색
            3: (0, 0, 255),  # 파란색
            4: (255, 255, 0),  # 사이안
            5: (255, 0, 255),  # 마젠타
        }
        return colors.get(robot_id, (255, 255, 255))  # 기본: 흰색

    def draw_apriltags(self):
        """AprilTag 및 로봇 그리기"""
        for detection in self.apriltag_detections:
            tag_id = detection["tag_id"]
            center = detection["center"]
            corners = detection["corners"]

            # 로봇인지 확인
            robot_id = self.get_robot_id_by_tag(tag_id)

            if robot_id is not None:
                # 로봇 그리기
                color = self.get_robot_color(robot_id)

                # 로봇 테두리 그리기 (두껍게)
                cv2.polylines(self.current_frame, [corners], True, color, 4)

                # 로봇 중심점 그리기 (크게)
                cv2.circle(self.current_frame, tuple(center), 8, color, -1)

                # yaw 방향 화살표 그리기
                if robot_id in self.robot_positions:
                    yaw = self.robot_positions[robot_id]["yaw"]
                    self.draw_yaw_arrow(center, yaw, color)

                # 로봇 ID 표시
                cv2.putText(
                    self.current_frame,
                    f"ROBOT {robot_id}",
                    (center[0] - 30, center[1] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    color,
                    2,
                )

                # 그리드 좌표 및 yaw 표시
                if len(self.corner_points) >= 4:
                    grid_pos = self.pos_to_grid(tuple(center))
                    if grid_pos:
                        row, col = grid_pos
                        cv2.putText(
                            self.current_frame,
                            f"G({row},{col})",
                            (center[0] - 30, center[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            color,
                            2,
                        )

                    real_coords = self.image_to_real_coords(center[0], center[1])
                    if real_coords:
                        real_x, real_y = real_coords
                        cv2.putText(
                            self.current_frame,
                            f"R({real_x:.2f},{real_y:.2f})",
                            (center[0] - 50, center[1] + 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color,
                            2,
                        )

                    # yaw 각도 표시
                    if robot_id in self.robot_positions:
                        yaw = self.robot_positions[robot_id]["yaw"]
                        cv2.putText(
                            self.current_frame,
                            f"Y:{yaw:.2f}rad",
                            (center[0] - 50, center[1] + 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            color,
                            2,
                        )
            else:
                # 일반 AprilTag 그리기
                cv2.polylines(self.current_frame, [corners], True, (0, 255, 255), 2)
                cv2.circle(self.current_frame, tuple(center), 5, (255, 0, 255), -1)
                cv2.putText(
                    self.current_frame,
                    f"TAG:{tag_id}",
                    (center[0] - 20, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2,
                )

    def image_to_real_coords(self, x, y):
        """이미지 좌표를 실제 좌표로 변환"""
        if len(self.corner_points) < 4:
            return None

        corners = np.array(self.corner_points[:4], dtype=np.float32)
        unit_square = np.array(
            [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=np.float32
        )

        try:
            matrix = cv2.getPerspectiveTransform(corners, unit_square)
            point = np.array([[[float(x), float(y)]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(point, matrix)

            u, v = transformed[0][0]
            real_x = u * self.real_width
            real_y = v * self.real_height

            if 0 <= u <= 1 and 0 <= v <= 1:
                return (real_x, real_y)
        except:
            pass

        return None

    def toggle_apriltag(self, state):
        """AprilTag 감지 활성화/비활성화"""
        self.enable_apriltag = state == Qt.CheckState.Checked.value
        if not self.enable_apriltag:
            self.apriltag_detections = []
            self.apriltag_status.setText("AprilTag: 비활성화")

    def draw_yaw_arrow(self, center, yaw, color):
        """로봇의 yaw 방향을 화살표로 표시"""
        arrow_length = 30
        arrow_end_x = int(center[0] + arrow_length * math.cos(yaw))
        arrow_end_y = int(center[1] + arrow_length * math.sin(yaw))

        # 메인 화살표 선
        cv2.arrowedLine(
            self.current_frame,
            tuple(center),
            (arrow_end_x, arrow_end_y),
            color,
            3,
            tipLength=0.3,
        )

    def create_robot_card(
        self, robot_id: int, robot_data: dict, status: RobotStatus = None
    ):
        """개별 로봇 카드 위젯 생성"""
        card = QFrame()
        card.setFixedSize(250, 90)

        is_online = False
        if self.status_subscriber:
            is_online = self.status_subscriber.is_robot_online(robot_id)
        if robot_data:  # detected by apriltag
            is_online = True

        border_color = "#9E9E9E"  # 오프라인
        if is_online:
            border_color = "#4CAF50"  # 온라인 기본
            if status and status.state:
                if status.state == "navigating":
                    border_color = "#4CAF50"  # 초록색 (이동 중)
                elif status.state == "idle":
                    border_color = "#2196F3"  # 파란색 (대기)
                elif status.state == "error":
                    border_color = "#F44336"  # 빨간색 (에러)
                else:
                    border_color = "#FF9800"  # 주황색 (기타)

        card.setStyleSheet(
            f"""
            QFrame {{
                background-color: white;
                border: 2px solid {border_color};
                border-radius: 6px;
                padding: 8px;
            }}
        """
        )

        layout = QVBoxLayout()
        layout.setSpacing(2)

        # 로봇 ID와 상태
        header_layout = QHBoxLayout()
        robot_label = QLabel(f"🤖 Robot {robot_id}")
        robot_font = QFont()
        robot_font.setBold(True)
        robot_font.setPointSize(10)
        robot_label.setFont(robot_font)
        header_layout.addWidget(robot_label)

        # 온라인 상태 표시
        if is_online:
            status_indicator = QLabel("🟢")
            status_indicator.setToolTip("온라인")
        else:
            status_indicator = QLabel("🔴")
            status_indicator.setToolTip("오프라인")
        header_layout.addWidget(status_indicator)
        header_layout.addStretch()

        # 배터리 표시
        if status and status.battery is not None:
            battery_level = status.battery * 100
            battery_icon = (
                "🔋" if battery_level > 50 else "🪫" if battery_level > 20 else "⚠️"
            )
            battery_label = QLabel(f"{battery_icon} {battery_level:.0f}%")
            battery_label.setStyleSheet("font-size: 9px;")
            header_layout.addWidget(battery_label)

        layout.addLayout(header_layout)

        # 위치 정보
        pos_text = "N/A"
        if robot_data and robot_data.get("real_coords"):
            real_x, real_y = robot_data["real_coords"]
            pos_text = f"카메라: ({real_x:.2f}, {real_y:.2f})"
        elif status and status.camera_pose:
            pos = status.camera_pose.pose.position
            pos_text = f"ROS: ({pos.x:.2f}, {pos.y:.2f})"

        pos_label = QLabel(f"📍 {pos_text}")
        pos_label.setStyleSheet("font-size: 9px; color: #666;")
        layout.addWidget(pos_label)

        # 상태 및 작업 정보
        info_layout = QHBoxLayout()
        if status and status.state:
            state_label = QLabel(f"[{status.state}]")
            state_label.setStyleSheet("font-size: 9px; color: #333; font-weight: bold;")
            info_layout.addWidget(state_label)

        if status and status.task:
            task_label = QLabel(f"📋 {status.task}")
            task_label.setStyleSheet("font-size: 8px; color: #666;")
            info_layout.addWidget(task_label)

        info_layout.addStretch()
        layout.addLayout(info_layout)

        layout.addStretch()
        card.setLayout(layout)

        return card

    def update_robot_list(self):
        """로봇 상태 카드 UI 업데이트"""
        if not hasattr(self, "robot_cards_layout"):
            return
        # 기존 카드들 제거
        for i in reversed(range(self.robot_cards_layout.count())):
            child = self.robot_cards_layout.takeAt(i).widget()
            if child:
                child.setParent(None)

        online_count = 0
        all_robot_ids = set(self.robot_positions.keys())

        # 상태 구독자에서 온라인 로봇들도 추가
        if self.status_subscriber:
            online_robots = [rid for rid in self.status_subscriber.robot_statuses.keys() 
                           if self.status_subscriber.is_robot_online(rid)]
            all_robot_ids.update(online_robots)

        total_robots = len(ROBOT_CONFIG)

        # 모든 로봇에 대해 카드 생성
        for robot_id in sorted(list(all_robot_ids)):
            robot_data = self.robot_positions.get(robot_id)
            status = None

            if self.status_subscriber:
                status = self.status_subscriber.robot_statuses.get(robot_id)

            is_online = (
                self.status_subscriber.is_robot_online(robot_id)
                if self.status_subscriber
                else False
            )
            if robot_data:
                is_online = True

            if is_online:
                online_count += 1

            # 로봇 카드 생성 및 추가
            card = self.create_robot_card(robot_id, robot_data, status)
            self.robot_cards_layout.addWidget(card)

        # 스트레치 추가 (카드들을 왼쪽 정렬)
        self.robot_cards_layout.addStretch()

        # 온라인 카운트 업데이트
        if hasattr(self, "online_count_label"):
            self.online_count_label.setText(f"온라인: {online_count}/{total_robots}대")

        # 카드 컨테이너 크기 업데이트
        card_count = self.robot_cards_layout.count()
        if card_count > 1:  # 1 is for the stretch
            if hasattr(self, "robot_cards_widget"):
                self.robot_cards_widget.setMinimumWidth((card_count - 1) * 260 + 50)

    def print_robot_positions(self):
        """콘솔에 로봇 위치 출력"""
        if self.robot_positions:
            for robot_id in sorted(self.robot_positions.keys()):
                robot_data = self.robot_positions[robot_id]
                grid_pos = robot_data["grid_pos"]
                real_coords = robot_data["real_coords"]
                tag_id = robot_data["tag_id"]
                yaw = robot_data["yaw"]

                if grid_pos and real_coords:
                    row, col = grid_pos
                    real_x, real_y = real_coords
                    print(
                        f"🤖 로봇{robot_id} (태그{tag_id}): 그리드({row},{col}) 실제({real_x:.3f},{real_y:.3f}) yaw:{yaw:.3f}rad ({math.degrees(yaw):.1f}°)"
                    )
                elif grid_pos:
                    row, col = grid_pos
                    print(
                        f"🤖 로봇{robot_id} (태그{tag_id}): 그리드({row},{col}) yaw:{yaw:.3f}rad ({math.degrees(yaw):.1f}°) - 그리드 미설정"
                    )
                else:
                    print(
                        f"🤖 로봇{robot_id} (태그{tag_id}): yaw:{yaw:.3f}rad ({math.degrees(yaw):.1f}°) 그리드 범위 외부"
                    )

            # ROS2 토픽으로 발행
            self.publish_robot_positions()

            # 웨이포인트 업데이트
            self.update_waypoint_following()

    def init_ros2(self):
        try:
            rclpy.init()
            self.ros_node = RobotStatusNode()
            self.status_subscriber = self.ros_node  # RobotStatusNode를 status_subscriber로 사용
            if self.ros_status:
                self.ros_status.setText("ROS2: 연결됨")
            print("✅ ROS2 노드 초기화 완료")
        except Exception as e:
            if self.ros_status:
                self.ros_status.setText(f"ROS2: 오류 - {str(e)}")
            print(f"❌ ROS2 초기화 실패: {e}")
            self.ros_node = None
            self.status_subscriber = None

    def spin_ros(self):
        """ROS2 스핀"""
        if self.ros_node:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.001)
            except Exception as e:
                print(f"ROS2 스핀 오류: {e}")

    def publish_robot_positions(self):
        """로봇 위치를 ROS2 토픽으로 발행"""
        if not self.ros_node:
            return

        for robot_id in self.robot_positions.keys():
            robot_data = self.robot_positions[robot_id]
            real_coords = robot_data["real_coords"]
            yaw = robot_data["yaw"]

            if real_coords:
                real_x, real_y = real_coords
                self.ros_node.publish_robot_pose(robot_id, real_x, real_y, yaw)

    def draw_robot_goals(self):
        """로봇 목표 그리기"""
        if len(self.corner_points) < 4:
            return

        corners = np.array(self.corner_points[:4], dtype=np.float32)

        for robot_id, goal_pos in self.robot_goals.items():
            row, col = goal_pos
            color = self.get_robot_color(robot_id)

            # 목표 위치 계산
            cell_corners = []
            for r_offset, c_offset in [(0, 0), (0, 1), (1, 1), (1, 0)]:
                cell_row = (row + r_offset) / self.grid_rows
                cell_col = (col + c_offset) / self.grid_cols

                top_point = corners[0] + cell_col * (corners[1] - corners[0])
                bottom_point = corners[2] + cell_col * (corners[3] - corners[2])
                cell_point = top_point + cell_row * (bottom_point - top_point)
                cell_corners.append(cell_point.astype(int))

            # 목표를 다이아몬드 모양으로 그리기
            pts = np.array(cell_corners)
            center = np.mean(pts, axis=0).astype(int)

            # 다이아몬드 모양 그리기
            diamond_size = 15
            diamond_pts = np.array(
                [
                    [center[0], center[1] - diamond_size],  # 위
                    [center[0] + diamond_size, center[1]],  # 오른쪽
                    [center[0], center[1] + diamond_size],  # 아래
                    [center[0] - diamond_size, center[1]],  # 왼쪽
                ]
            )

            cv2.fillPoly(self.current_frame, [diamond_pts], color)
            cv2.polylines(self.current_frame, [diamond_pts], True, (255, 255, 255), 2)

            # 로봇 ID 표시
            cv2.putText(
                self.current_frame,
                f"G{robot_id}",
                (center[0] - 10, center[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

    def draw_robot_paths(self):
        """로봇 경로 그리기"""
        if len(self.corner_points) < 4 or not self.enable_path_planning:
            return

        corners = np.array(self.corner_points[:4], dtype=np.float32)

        for robot_id, path in self.robot_paths.items():
            if len(path) <= 1:
                continue

            color = self.get_robot_color(robot_id)

            # 경로 선 그리기
            path_points = []
            for row, col in path:
                # 그리드 셀 중심 계산
                cell_row = (row + 0.5) / self.grid_rows
                cell_col = (col + 0.5) / self.grid_cols

                top_point = corners[0] + cell_col * (corners[1] - corners[0])
                bottom_point = corners[2] + cell_col * (corners[3] - corners[2])
                cell_center = top_point + cell_row * (bottom_point - top_point)
                path_points.append(cell_center.astype(int))

            # 경로 선 그리기
            for i in range(len(path_points) - 1):
                cv2.line(
                    self.current_frame,
                    tuple(path_points[i]),
                    tuple(path_points[i + 1]),
                    color,
                    3,
                )

                # 방향 화살표 그리기
                if i < len(path_points) - 1:
                    start_pt = path_points[i]
                    end_pt = path_points[i + 1]
                    cv2.arrowedLine(
                        self.current_frame,
                        tuple(start_pt),
                        tuple(end_pt),
                        color,
                        2,
                        tipLength=0.3,
                    )

    def start_goal_setting(self):
        """목표 설정 모드 시작"""
        robot_index = self.robot_goal_combo.currentIndex()
        robot_ids = list(ROBOT_CONFIG.keys())
        if robot_index < len(robot_ids):
            self.setting_goal_for_robot = robot_ids[robot_index]
            self.set_goal_button.setText(
                f"로봇{self.setting_goal_for_robot} 목표 설정 중..."
            )

    def toggle_path_planning(self, state):
        """경로 계획 활성화/비활성화"""
        self.enable_path_planning = state == Qt.CheckState.Checked.value
        if not self.enable_path_planning:
            self.robot_paths = {}

    def plan_robot_paths(self):
        """로봇 경로 계획 실행"""
        if not self.enable_path_planning or not self.robot_goals:
            return

        # 현재 로봇 위치 추출
        current_positions = {}
        for robot_id in self.robot_positions.keys():
            robot_data = self.robot_positions[robot_id]
            grid_pos = robot_data["grid_pos"]
            if grid_pos:
                current_positions[robot_id] = grid_pos

        if not current_positions:
            print("⚠️ 로봇 위치를 찾을 수 없습니다.")
            return

        # 경로 계획자에 장애물 및 목표 설정
        self.path_planner.set_obstacles(self.obstacles)
        for robot_id, goal in self.robot_goals.items():
            self.path_planner.set_robot_goal(robot_id, goal)

        # 경로 계획 실행
        planned_paths = self.path_planner.plan_multi_robot_paths(current_positions)
        self.robot_paths = planned_paths

        # 결과 출력 및 실제 주행
        for robot_id, path in planned_paths.items():
            if len(path) > 1:
                print(
                    f"🛣️ 로봇{robot_id} 경로: {path[:5]}{'...' if len(path) > 5 else ''} (total: {len(path)} steps)"
                )

                # A* 경로를 실제 주행용 웨이포인트로 변환 및 발행
                if self.enable_robot_control:
                    self.publish_path_as_waypoints(robot_id, path)
            else:
                print(f"🛣️ 로봇{robot_id}: 목표에 도달하였거나 경로를 찾을 수 없습니다.")

    def clear_all_paths(self):
        """모든 경로 지우기"""
        self.robot_paths = {}
        self.robot_goals = {}
        self.setting_goal_for_robot = None
        self.set_goal_button.setText("목표 설정")
        print("🗑️ 모든 경로와 목표가 지워졌습니다.")

    def clear_obstacles(self):
        """모든 장애물 제거"""
        self.obstacles.clear()
        self.update_obstacle_list()

    def set_robot_waypoints(self, robot_id, waypoints):
        """로봇에 웨이포인트 리스트 설정
        Args:
            robot_id: 로봇 ID
            waypoints: [(x, y), ...] 실제 좌표계의 웨이포인트 리스트
        """
        self.robot_waypoints[robot_id] = waypoints.copy()
        self.robot_current_waypoint_index[robot_id] = 0
        self.update_waypoint_list()
        print(f"🎯 로봇{robot_id}에 {len(waypoints)}개 웨이포인트 설정: {waypoints}")

    def set_test_waypoints(self):
        """테스트용 웨이포인트 설정"""
        # 로봇 1에 대한 테스트 웨이포인트 (실제 좌표)
        test_waypoints_robot1 = [
            (0.5, 0.2),  # 첫 번째 웨이포인트
            (1.2, 0.3),  # 두 번째 웨이포인트
            (1.5, 0.7),  # 세 번째 웨이포인트
            (0.8, 0.8),  # 마지막 웨이포인트
        ]

        # 로봇 2에 대한 테스트 웨이포인트 (실제 좌표)
        test_waypoints_robot2 = [
            (1.8, 0.1),  # 첫 번째 웨이포인트
            (1.0, 0.4),  # 두 번째 웨이포인트
            (0.3, 0.6),  # 세 번째 웨이포인트
        ]

        if 1 in ROBOT_CONFIG:
            self.set_robot_waypoints(1, test_waypoints_robot1)
        if 2 in ROBOT_CONFIG:
            self.set_robot_waypoints(2, test_waypoints_robot2)

    def clear_waypoints(self):
        """모든 웨이포인트 지우기"""
        self.robot_waypoints = {}
        self.robot_current_waypoint_index = {}
        self.robot_target_published = {}
        self.update_waypoint_list()
        print("🗑️ 모든 웨이포인트가 지워졌습니다.")

    def update_waypoint_list(self):
        """웨이포인트 리스트 UI 업데이트"""
        self.waypoint_list.clear()
        for robot_id, waypoints in self.robot_waypoints.items():
            current_index = self.robot_current_waypoint_index.get(robot_id, 0)
            for i, (x, y) in enumerate(waypoints):
                status = (
                    "✓" if i < current_index else "→" if i == current_index else "○"
                )
                item_text = f"로봇{robot_id} [{status}] WP{i+1}: ({x:.2f}, {y:.2f})"
                self.waypoint_list.addItem(item_text)

    def update_waypoint_following(self):
        """웨이포인트 팔로잉 업데이트"""
        for robot_id, waypoints in self.robot_waypoints.items():
            if robot_id not in self.robot_positions:
                continue

            current_index = self.robot_current_waypoint_index.get(robot_id, 0)
            if current_index >= len(waypoints):
                continue  # 모든 웨이포인트 완료

            # 현재 로봇 위치 (실제 좌표)
            robot_data = self.robot_positions[robot_id]
            real_coords = robot_data.get("real_coords")
            if not real_coords:
                continue

            robot_x, robot_y = real_coords
            target_x, target_y = waypoints[current_index]

            # 목표까지의 거리 계산
            distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)

            if distance <= self.waypoint_tolerance:
                # 웨이포인트에 도달
                print(
                    f"✅ 로봇{robot_id} 웨이포인트 {current_index+1} 도달: ({target_x:.2f}, {target_y:.2f})"
                )
                self.robot_current_waypoint_index[robot_id] = current_index + 1
                self.robot_target_published[robot_id] = False  # 다음 목표를 위해 리셋

                if current_index + 1 >= len(waypoints):
                    print(f"🏁 로봇{robot_id} 모든 웨이포인트 완료!")
                    # 로봇 정지
                    if self.enable_robot_control and self.ros_node:
                        self.ros_node.stop_robot(robot_id)
                else:
                    # 다음 웨이포인트로 경로 계획
                    next_waypoint = waypoints[current_index + 1]
                    self.plan_to_waypoint(robot_id, next_waypoint)

                    # 다음 웨이포인트를 ROS2로 발행
                    if self.enable_robot_control and self.ros_node:
                        next_x, next_y = next_waypoint
                        self.ros_node.publish_target_pose(robot_id, next_x, next_y)
                        self.robot_target_published[robot_id] = True

                self.update_waypoint_list()
            else:
                # 아직 도달하지 않았으면 현재 웨이포인트로 경로 계획 및 ROS2 발행
                if current_index == 0 or not self.robot_paths.get(robot_id):
                    self.plan_to_waypoint(robot_id, waypoints[current_index])

                # 현재 웨이포인트를 ROS2로 발행 (한 번만)
                if (
                    self.enable_robot_control
                    and self.ros_node
                    and not self.robot_target_published.get(robot_id, False)
                ):
                    self.ros_node.publish_target_pose(robot_id, target_x, target_y)
                    self.robot_target_published[robot_id] = True

    def plan_to_waypoint(self, robot_id, target_waypoint):
        """특정 웨이포인트로의 경로 계획"""
        if robot_id not in self.robot_positions:
            return

        robot_data = self.robot_positions[robot_id]
        grid_pos = robot_data.get("grid_pos")
        if not grid_pos:
            return

        # 목표 웨이포인트를 그리드 좌표로 변환
        target_grid = self.real_coords_to_grid(target_waypoint[0], target_waypoint[1])
        if not target_grid:
            return

        # 경로 계획자 설정
        self.path_planner.set_obstacles(self.obstacles)
        self.path_planner.set_robot_goal(robot_id, target_grid)

        # 단일 로봇에 대한 경로 계획
        planned_paths = self.path_planner.plan_multi_robot_paths({robot_id: grid_pos})

        if robot_id in planned_paths and len(planned_paths[robot_id]) > 1:
            self.robot_paths[robot_id] = planned_paths[robot_id]
            print(
                f"🛣️ 로봇{robot_id} 웨이포인트 경로 계획: {len(planned_paths[robot_id])} 스텝"
            )

            # A* 경로를 실제 웨이포인트로 변환하여 발행
            if self.enable_robot_control:
                self.publish_path_as_waypoints(robot_id, planned_paths[robot_id])
        else:
            print(f"⚠️ 로봇{robot_id} 웨이포인트 경로를 찾을 수 없습니다.")

    def real_coords_to_grid(self, real_x, real_y):
        """실제 좌표를 그리드 좌표로 변환"""
        if (
            real_x < 0
            or real_x > self.real_width
            or real_y < 0
            or real_y > self.real_height
        ):
            return None

        grid_col = int(real_x / self.real_width * self.grid_cols)
        grid_row = int(real_y / self.real_height * self.grid_rows)

        # 범위 체크
        grid_col = max(0, min(grid_col, self.grid_cols - 1))
        grid_row = max(0, min(grid_row, self.grid_rows - 1))

        return (grid_row, grid_col)

    def draw_waypoints(self):
        """웨이포인트 시각화"""
        if len(self.corner_points) < 4:
            return

        for robot_id, waypoints in self.robot_waypoints.items():
            current_index = self.robot_current_waypoint_index.get(robot_id, 0)
            color = self.get_robot_color(robot_id)

            for i, (real_x, real_y) in enumerate(waypoints):
                # 실제 좌표를 이미지 좌표로 변환
                image_pos = self.real_coords_to_image_coords(real_x, real_y)
                if not image_pos:
                    continue

                x, y = image_pos

                # 웨이포인트 상태에 따른 색상
                if i < current_index:
                    # 완료된 웨이포인트 (회색)
                    wp_color = (128, 128, 128)
                elif i == current_index:
                    # 현재 목표 웨이포인트 (밝은 색상)
                    wp_color = color
                else:
                    # 대기 중인 웨이포인트 (어두운 색상)
                    wp_color = tuple(c // 2 for c in color)

                # 웨이포인트를 원으로 그리기
                cv2.circle(self.current_frame, (int(x), int(y)), 10, wp_color, -1)
                cv2.circle(self.current_frame, (int(x), int(y)), 10, (255, 255, 255), 2)

                # 웨이포인트 번호 표시
                cv2.putText(
                    self.current_frame,
                    f"WP{i+1}",
                    (int(x) - 15, int(y) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )

                # 로봇 ID 표시 (첫 번째 웨이포인트에만)
                if i == 0:
                    cv2.putText(
                        self.current_frame,
                        f"R{robot_id}",
                        (int(x) - 10, int(y) + 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                    )

            # 웨이포인트 간 연결선 그리기
            if len(waypoints) > 1:
                for i in range(len(waypoints) - 1):
                    pos1 = self.real_coords_to_image_coords(
                        waypoints[i][0], waypoints[i][1]
                    )
                    pos2 = self.real_coords_to_image_coords(
                        waypoints[i + 1][0], waypoints[i + 1][1]
                    )

                    if pos1 and pos2:
                        line_color = tuple(c // 2 for c in color)  # 어두운 색상
                        cv2.line(
                            self.current_frame,
                            (int(pos1[0]), int(pos1[1])),
                            (int(pos2[0]), int(pos2[1])),
                            line_color,
                            2,
                        )

    def real_coords_to_image_coords(self, real_x, real_y):
        """실제 좌표를 이미지 좌표로 변환"""
        if len(self.corner_points) < 4:
            return None

        # 실제 좌표를 정규화된 좌표로 변환
        u = real_x / self.real_width
        v = real_y / self.real_height

        if not (0 <= u <= 1 and 0 <= v <= 1):
            return None

        # 정규화된 좌표를 이미지 좌표로 변환
        corners = np.array(self.corner_points[:4], dtype=np.float32)
        unit_square = np.array(
            [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=np.float32
        )

        try:
            # 역변환 매트릭스
            matrix = cv2.getPerspectiveTransform(unit_square, corners)
            point = np.array([[[u, v]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(point, matrix)

            x, y = transformed[0][0]
            return (x, y)
        except:
            return None

    def publish_path_as_waypoints(self, robot_id, path):
        """A* 경로를 순차 웨이포인트로 변환하여 발행"""
        if not self.ros_node or len(path) <= 1:
            return

        # 경로의 각 그리드 포인트를 실제 좌표로 변환
        path_waypoints = []
        for row, col in path[1:]:  # 현재 위치 제외
            real_coords = self.grid_to_real_coords(row, col)
            if real_coords:
                path_waypoints.append(real_coords)

        if not path_waypoints:
            return

        # 첫 번째 경로 포인트를 즉시 발행
        first_waypoint = path_waypoints[0]
        self.ros_node.publish_target_pose(
            robot_id, first_waypoint[0], first_waypoint[1]
        )

        # 나머지 경로 포인트들을 로봇 경로 시스템에 저장
        if len(path_waypoints) > 1:
            # 기존 웨이포인트를 경로 기반으로 업데이트
            remaining_waypoints = path_waypoints[1:]
            print(
                f"📍 로봇{robot_id} A* 경로 기반 웨이포인트 {len(remaining_waypoints)}개 추가 대기"
            )

        # 목표 발행 상태 업데이트
        self.robot_target_published[robot_id] = True

    def toggle_robot_control(self, state):
        """로봇 자동 제어 활성화/비활성화"""
        self.enable_robot_control = state == Qt.CheckState.Checked.value
        if self.enable_robot_control:
            print("🤖 로봇 자동 제어가 활성화되었습니다.")
        else:
            print("⏹️ 로봇 자동 제어가 비활성화되었습니다.")
            # 모든 로봇 정지
            self.stop_all_robots()

    def start_waypoint_mission(self):
        """웨이포인트 미션 시작"""
        if not self.robot_waypoints:
            print("⚠️ 설정된 웨이포인트가 없습니다.")
            return

        if not self.enable_robot_control:
            print("⚠️ 먼저 로봇 자동 제어를 활성화하세요.")
            return

        # 모든 로봇의 웨이포인트 인덱스를 처음으로 초기화
        for robot_id in self.robot_waypoints.keys():
            self.robot_current_waypoint_index[robot_id] = 0
            self.robot_target_published[robot_id] = False

        self.update_waypoint_list()
        print("🚀 웨이포인트 미션을 시작합니다!")

        # 첫 번째 웨이포인트들을 각 로봇에 발행
        for robot_id, waypoints in self.robot_waypoints.items():
            if waypoints and robot_id in self.robot_positions:
                first_waypoint = waypoints[0]
                if self.ros_node:
                    self.ros_node.publish_target_pose(
                        robot_id, first_waypoint[0], first_waypoint[1]
                    )
                    self.robot_target_published[robot_id] = True
                    print(
                        f"📡 로봇{robot_id}에 첫 번째 웨이포인트 전송: {first_waypoint}"
                    )

    def stop_all_robots(self):
        """모든 로봇 정지"""
        if self.ros_node:
            for robot_id in ROBOT_CONFIG.keys():
                self.ros_node.stop_robot(robot_id)
            print("🛑 모든 로봇이 정지되었습니다.")

        # 목표 발행 상태 초기화
        self.robot_target_published = {}

    def closeEvent(self, event):
        """앱 종료시 카메라 및 ROS2 정리"""
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()

        # ROS2 정리
        if self.ros_node:
            self.ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

        event.accept()


class ClickableLabel(QLabel):
    """클릭 가능한 QLabel"""

    mouse_clicked = Signal(QPoint)

    def __init__(self):
        super().__init__()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.mouse_clicked.emit(event.position().toPoint())


def main():
    app = QApplication(sys.argv)
    window = GridCameraApp()
    window.show()

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("프로그램 종료")


if __name__ == "__main__":
    main()
