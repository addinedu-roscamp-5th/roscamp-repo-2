#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2, sin, cos, sqrt, pi

import cv2
import numpy as np
from pupil_apriltags import Detector

import sys


# === 설정 ===
TAG_ID = 2
CAMERA_INDEX = 2
VISUALIZE = True

# === 그리드 설정 ===
CAP_WIDTH = 1920
CAP_HEIGHT = 1080

TOP_LEFT = (0, 0)
TOP_RIGHT = (CAP_WIDTH - 1, 0)
BOT_LEFT = (0, CAP_HEIGHT - 1)
BOT_RIGHT = (CAP_WIDTH - 1, CAP_HEIGHT - 1)

ROWS, COLS = 12, 24
REAL_MAX_WIDTH = 1.91
REAL_MAX_HEIGHT = 0.91
REAL_ROWS = ROWS + 1
REAL_COLS = COLS + 1
REAL_WIDTH = REAL_MAX_WIDTH / REAL_COLS
REAL_HEIGHT = REAL_MAX_HEIGHT / REAL_ROWS

horizontal_divisions = ROWS - 1
vertical_divisions = COLS - 1

# === 로봇 제어 설정 ===
MAX_LINEAR_SPEED = 0.3  # m/s
MAX_ANGULAR_SPEED = 1.0  # rad/s
POSITION_TOLERANCE = 0.05  # m
ANGLE_TOLERANCE = 0.1  # rad

# === ROS 노드 클래스 ===
class PoseMultiPublisher(Node):
    def __init__(self, robot_ids, get_pose_func_map, publish_period=0.03):
        super().__init__("multi_pose_publisher")
        self.pose_publishers = {}  # {id: publisher}
        self.cmd_publishers = {}   # {id: cmd_vel publisher}
        self.get_pose_func_map = get_pose_func_map  # {id: func}
        
        # 로봇 제어 상태
        self.robot_goals = {}  # {robot_id: (target_x, target_y, target_yaw)}
        self.robot_paths = {}  # {robot_id: [(x1, y1), (x2, y2), ...]}
        self.path_index = {}   # {robot_id: current_path_index}
        self.robot_states = {} # {robot_id: 'idle'/'moving'/'rotating'}

        for robot_id in robot_ids:
            # Pose publisher
            pose_topic = f"/robot{robot_id}/camera_pose"
            self.pose_publishers[robot_id] = self.create_publisher(
                PoseStamped, pose_topic, 10
            )
            
            # Command velocity publisher
            cmd_topic = f"/robot{robot_id}/cmd_vel"
            self.cmd_publishers[robot_id] = self.create_publisher(
                Twist, cmd_topic, 10
            )
            
            # 제어 상태 초기화
            self.robot_goals[robot_id] = None
            self.robot_paths[robot_id] = []
            self.path_index[robot_id] = 0
            self.robot_states[robot_id] = 'idle'
            
            # 타이머 생성
            self.create_timer(
                publish_period, lambda rid=robot_id: self.publish_pose(rid)
            )
            self.create_timer(
                0.1, lambda rid=robot_id: self.control_robot(rid)
            )

    def set_robot_goal(self, robot_id, target_x, target_y, target_yaw=None):
        """로봇의 목표 위치 설정"""
        if target_yaw is None:
            # 현재 위치에서 목표까지의 방향으로 yaw 계산
            current_pose = self.get_pose_func_map[robot_id]()
            if current_pose:
                current_x, current_y, _ = current_pose
                target_yaw = atan2(target_y - current_y, target_x - current_x)
            else:
                target_yaw = 0.0
                
        self.robot_goals[robot_id] = (target_x, target_y, target_yaw)
        self.robot_states[robot_id] = 'moving'
        self.get_logger().info(
            f"🎯 로봇{robot_id} 목표 설정: x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f}"
        )

    def set_robot_path(self, robot_id, waypoints):
        """로봇의 경로 설정 (여러 웨이포인트)"""
        self.robot_paths[robot_id] = waypoints
        self.path_index[robot_id] = 0
        if waypoints:
            first_point = waypoints[0]
            self.set_robot_goal(robot_id, first_point[0], first_point[1])

    def stop_robot(self, robot_id):
        """로봇 정지"""
        self.robot_goals[robot_id] = None
        self.robot_paths[robot_id] = []
        self.robot_states[robot_id] = 'idle'
        
        # 정지 명령 전송
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_publishers[robot_id].publish(cmd)
        
        self.get_logger().info(f"🛑 로봇{robot_id} 정지")

    def control_robot(self, robot_id):
        """로봇 제어 루프"""
        if self.robot_states[robot_id] == 'idle' or self.robot_goals[robot_id] is None:
            return
            
        current_pose = self.get_pose_func_map[robot_id]()
        if not current_pose:
            return
            
        current_x, current_y, current_yaw = current_pose
        target_x, target_y, target_yaw = self.robot_goals[robot_id]
        
        # 목표까지의 거리와 각도 계산
        dx = target_x - current_x
        dy = target_y - current_y
        distance = sqrt(dx*dx + dy*dy)
        target_angle = atan2(dy, dx)
        
        cmd = Twist()
        
        # 목표 도달 확인
        if distance < POSITION_TOLERANCE:
            # 위치 도달, 다음 웨이포인트 확인
            if self.robot_paths[robot_id] and self.path_index[robot_id] < len(self.robot_paths[robot_id]) - 1:
                # 다음 웨이포인트로 이동
                self.path_index[robot_id] += 1
                next_point = self.robot_paths[robot_id][self.path_index[robot_id]]
                self.set_robot_goal(robot_id, next_point[0], next_point[1])
                return
            else:
                # 최종 목표 도달, 회전 조정
                angle_diff = self.normalize_angle(target_yaw - current_yaw)
                if abs(angle_diff) > ANGLE_TOLERANCE:
                    cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angle_diff * 2.0))
                else:
                    # 완전히 도달
                    self.stop_robot(robot_id)
                    self.get_logger().info(f"✅ 로봇{robot_id} 목표 도달!")
                    return
        else:
            # 목표로 이동
            angle_diff = self.normalize_angle(target_angle - current_yaw)
            
            # 각도 조정이 필요한 경우
            if abs(angle_diff) > 0.3:  # 약 17도
                cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angle_diff * 2.0))
                cmd.linear.x = MAX_LINEAR_SPEED * 0.3  # 회전 중에는 천천히
            else:
                # 직진
                cmd.linear.x = min(MAX_LINEAR_SPEED, distance * 2.0)
                cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angle_diff * 1.0))
        
        self.cmd_publishers[robot_id].publish(cmd)

    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 범위로 정규화"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def publish_pose(self, robot_id):
        pose_func = self.get_pose_func_map[robot_id]
        pose = pose_func()
        if pose is None:
            return

        x, y, yaw = pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.pose_publishers[robot_id].publish(msg)

# === 웹캠 추적 스레드 ===
class WebcamThread(threading.Thread):
    def __init__(self, tag_id, camera_index=0, visualize=False, frame_callback=None):
        super().__init__()
        self.frame_callback = frame_callback
        self.tag_id = tag_id
        self.camera_index = camera_index
        self.visualize = visualize
        self.result_lock = threading.Lock()
        self.pose = None
        self.running = True
        self.p0 = None
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.printed_once = True
        self.grid_corners = None
        self.all_tag_poses = []

    # 보간 함수
    def interpolate(self, p1, p2, t):
        return (int(p1[0] + t * (p2[0] - p1[0])), int(p1[1] + t * (p2[1] - p1[1])))

    def generate_grid_points(self, h_div, v_div):
        if self.grid_corners is None or len(self.grid_corners) != 4:
            return []

        lefts = [
            self.interpolate(self.p0, self.p2, i / h_div) for i in range(h_div + 1)
        ]
        rights = [
            self.interpolate(self.p1, self.p3, i / h_div) for i in range(h_div + 1)
        ]

        tops = [self.interpolate(self.p0, self.p1, i / v_div) for i in range(v_div + 1)]
        bottoms = [
            self.interpolate(self.p2, self.p3, i / v_div) for i in range(v_div + 1)
        ]

        grid_points = []
        for row, (l, r) in enumerate(zip(lefts, rights)):
            for col, (t, b) in enumerate(zip(tops, bottoms)):
                pt = self.interpolate(l, r, col / v_div)
                grid_points.append((row, col, pt))

        return grid_points

    def visualize_grid(self, img, grid_points, rows, cols):

        # 가로줄
        for row in range(rows):
            pts = [pt for r, c, pt in grid_points if r == row]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (255, 255, 0), 2)

        # 세로줄
        for col in range(cols):
            pts = [pt for r, c, pt in grid_points if c == col]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (0, 255, 255), 2)

        # 점
        for row, col, pt in grid_points:
            cv2.circle(img, pt, 2, (0, 0, 255), 2)

    # 📍 각 꼭짓점에서 가장 가까운 실제 에이프릴 태그 중심 좌표로 snap
    def find_closest(self, center):
        min_dist = float("inf")
        closest = center
        for c in self.all_tag_poses:
            c_pt = c["pose"][:2]  # (x, y)만 사용
            dist = np.linalg.norm(np.array(c_pt) - np.array(center))
            if dist < min_dist:
                min_dist = dist
                closest = c_pt
        return tuple(closest)

    def get_tag_pose(self, tag_id):
        with self.result_lock:
            return [
                entry["pose"] for entry in self.all_tag_poses if entry["id"] == tag_id
            ]

    def get_all_tag_poses(self):
        with self.result_lock:
            return list(self.all_tag_poses)  # 사본 반환

    def draw_grid_corners(self, frame):
        # p0~p3가 None이면 새로 할당
        if None in (self.p0, self.p1, self.p2, self.p3):
            self.p0 = self.find_closest(TOP_LEFT)
            self.p1 = self.find_closest(TOP_RIGHT)
            self.p2 = self.find_closest(BOT_LEFT)
            self.p3 = self.find_closest(BOT_RIGHT)

        self.grid_corners = [self.p0, self.p1, self.p2, self.p3]
        if self.grid_corners:
            grid_points = self.generate_grid_points(
                horizontal_divisions, vertical_divisions
            )
            self.visualize_grid(frame, grid_points, ROWS, COLS)

    def redraw_grid_corners(self, frame):
        # p0~p3가 None이면 새로 할당
        self.p0 = self.find_closest(TOP_LEFT)
        self.p1 = self.find_closest(TOP_RIGHT)
        self.p2 = self.find_closest(BOT_LEFT)
        self.p3 = self.find_closest(BOT_RIGHT)

        self.grid_corners = [self.p0, self.p1, self.p2, self.p3]
        if self.grid_corners:
            grid_points = self.generate_grid_points(
                horizontal_divisions, vertical_divisions
            )
            self.visualize_grid(frame, grid_points, ROWS, COLS)

    # ✅ 헬퍼 함수: tag_id에 해당하는 pose를 가져와 변환
    def get_transed_pose_for_tag(self, tag_id):
        poses = self.get_tag_pose(tag_id)
        if not poses:
            return None
        # 여러 개일 수 있으므로 첫 번째만 사용
        x, y, yaw = poses[0]

        real_x, real_y = self.transed_point(x,y)

        return real_x, real_y, yaw
        

    def transed_point(self, x, y):
        # 변환
        if self.grid_corners is not None:
            grid_corners = np.array(
                self.grid_corners,
                dtype=np.float32,
            )
        unit_square = np.array(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
            ],
            dtype=np.float32,
        )
        try:
            Minv = cv2.getPerspectiveTransform(grid_corners, unit_square)
        except cv2.error as e:
            print("⚠️ Perspective 변환 실패:", e)
            return None
        
        img_point = np.array([[[x, y]]], dtype=np.float32)
        relative_point = cv2.perspectiveTransform(img_point, Minv)
        u, v = relative_point[0][0]
        real_x = u * REAL_MAX_WIDTH
        real_y = REAL_MAX_HEIGHT - v * REAL_MAX_HEIGHT
        return real_x, real_y

    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)

        if not cap.isOpened():
            print("❌ 웹캠 열기 실패")
            return

        detector = Detector(families="tag36h11")
        print(f"🎯 AprilTag ID {self.tag_id} 추적 중...")

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("오류: 프레임을 읽을 수 없습니다.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray, estimate_tag_pose=False)

            current_tag_poses_list = []
            for tag in tags:
                cX, cY = int(tag.center[0]), int(tag.center[1])
                pt0, pt1 = tag.corners[0], tag.corners[1]
                dx = pt1[0] - pt0[0]
                dy = pt1[1] - pt0[1]
                yaw = atan2(dy, dx)

                # ▶️ AprilTag 사각형 그리기
                corners = np.array(tag.corners, dtype=np.int32)
                cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                # ▶️ x축 방향 그리기 (화살표)
                arrow_len = 40  # 화살표 길이
                arrow_tip = (int(cX + arrow_len * cos(yaw)), int(cY + arrow_len * sin(yaw)))
                cv2.arrowedLine(frame, (cX, cY), arrow_tip, (0, 0, 255), 3, tipLength=0.3)
                # 빨간색 (0,0,255) 으로 표시됨

                # 🏷 ID 값 텍스트로 출력
                tag_id_text = f"ID: {tag.tag_id}"
                cv2.putText(frame, tag_id_text, (cX - 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(frame, tag_id_text, (cX - 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 📝 내부 데이터로 저장
                current_tag_poses_list.append(
                    {
                        "id": tag.tag_id,
                        "pose": (cX, cY, -yaw),
                        "corners": [(int(p[0]), int(p[1])) for p in tag.corners],
                    }
                )

                # 📍 타겟 태그일 경우 pose 저장
                if tag.tag_id == self.tag_id:
                    with self.result_lock:
                        self.pose = (cX, cY, -yaw)


            with self.result_lock:
                self.all_tag_poses = current_tag_poses_list

            if self.all_tag_poses:
                if self.visualize:
                    self.draw_grid_corners(frame)

            if self.frame_callback:
                self.frame_callback(frame)  # 원본 크기 그대로 전달

        cap.release()
        cv2.destroyAllWindows()
        print("🛑 웹캠 스레드 종료")

    def stop(self):
        self.running = False


from PySide6.QtWidgets import QLabel
from PySide6.QtCore import Qt, Signal, QPoint

class ImageLabel(QLabel):
    mouse_clicked = Signal(QPoint)  # 📌 클릭 위치를 신호로 보냄
    mouse_moved = Signal(QPoint)    # (이미 있는 마우스 이동 신호 유지)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)

    def mouseMoveEvent(self, event):
        self.mouse_moved.emit(event.position().toPoint())  # 마우스 이동 시 위치 전달

    def mousePressEvent(self, event):
        self.mouse_clicked.emit(event.position().toPoint())  # 마우스 클릭 시 위치 전달


from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, 
    QSizePolicy, QListWidget, QListWidgetItem, QLineEdit, QCheckBox, QComboBox,
    QTextEdit, QScrollArea, QFrame
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, Signal

class ImageWindow(QWidget):
    def __init__(self, robot_tag_map):
        super().__init__()
        self.setWindowTitle("AprilTag Robot Controller")
        self.setGeometry(100, 100, 1400, 800)

        self.robot_tag_map = robot_tag_map
        self.selected_robot_id = None
        self.cursor_pos = None
        self.webcam_thread = None
        self.current_frame = None
        self.ros_node = None
        
        # 경로 계획 상태
        self.waypoints = {}  # {robot_id: [(x1, y1), (x2, y2), ...]}
        self.start_points = {}  # {robot_id: (x, y)}
        self.goal_points = {}   # {robot_id: (x, y)}
        self.path_planning_mode = False
        self.current_waypoint_robot = None

        self.setup_ui()

    def setup_ui(self):
        # ▶ 로봇 리스트 (왼쪽 위)
        self.robot_list = QListWidget()
        self.robot_list.setFixedWidth(250)
        self.robot_list.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self.robot_list.itemClicked.connect(self.handle_robot_select)

        for robot_id in self.robot_tag_map:
            item = QListWidgetItem(f"🤖 로봇-{robot_id}")
            self.robot_list.addItem(item)

        # ▶ 태그 정보 폼
        self.tag_info_label = QLabel("태그 ID:")
        self.tag_edit = QLineEdit()
        self.tag_edit.setFixedWidth(100)
        self.save_button = QPushButton("💾 저장")
        self.save_button.setFixedWidth(100)
        self.save_button.clicked.connect(self.save_tag_id)
        
        self.tag_x_label = QLabel("X:")
        self.tag_x = QLineEdit()
        self.tag_x.setFixedWidth(100)
        self.tag_y_label = QLabel("Y:")
        self.tag_y = QLineEdit()
        self.tag_y.setFixedWidth(100)
        self.tag_yaw_label = QLabel("Yaw:")
        self.tag_yaw = QLineEdit()
        self.tag_yaw.setFixedWidth(100)

        # ▶ 로봇 제어 섹션
        control_frame = QFrame()
        control_frame.setFrameStyle(QFrame.Shape.Box)
        
        self.control_label = QLabel("🎮 로봇 제어")
        self.control_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        
        # 제어 모드 선택
        self.mode_label = QLabel("제어 모드:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["단일 목표", "경로 계획", "수동 제어"])
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        
        # 목표 설정 버튼들
        self.set_start_btn = QPushButton("🎯 시작점 설정")
        self.set_start_btn.clicked.connect(self.set_start_point_mode)
        self.set_start_btn.setEnabled(False)
        
        self.set_goal_btn = QPushButton("🏁 목표점 설정")
        self.set_goal_btn.clicked.connect(self.set_goal_point_mode)
        self.set_goal_btn.setEnabled(False)
        
        self.add_waypoint_btn = QPushButton("📍 웨이포인트 추가")
        self.add_waypoint_btn.clicked.connect(self.add_waypoint_mode)
        self.add_waypoint_btn.setEnabled(False)
        
        self.execute_btn = QPushButton("▶️ 실행")
        self.execute_btn.clicked.connect(self.execute_path)
        self.execute_btn.setEnabled(False)
        
        self.stop_btn = QPushButton("⏹️ 정지")
        self.stop_btn.clicked.connect(self.stop_robot)
        self.stop_btn.setEnabled(False)
        
        self.clear_path_btn = QPushButton("🗑️ 경로 지우기")
        self.clear_path_btn.clicked.connect(self.clear_path)

        # 경로 정보 표시
        self.path_info = QTextEdit()
        self.path_info.setMaximumHeight(100)
        self.path_info.setReadOnly(True)

        # 리셋 버튼
        self.refresh_button = QPushButton("🔄 그리드 리셋")
        self.refresh_button.clicked.connect(self.handle_refresh_click)

        # 레이아웃 구성
        tag_form = QVBoxLayout()
        tag_form.addWidget(self.tag_info_label)
        tag_form.addWidget(self.tag_edit)
        tag_form.addWidget(self.save_button)

        pos_layout = QHBoxLayout()
        pos_layout.addWidget(self.tag_x_label)
        pos_layout.addWidget(self.tag_x)
        pos_layout.addWidget(self.tag_y_label)
        pos_layout.addWidget(self.tag_y)

        yaw_layout = QHBoxLayout()
        yaw_layout.addWidget(self.tag_yaw_label)
        yaw_layout.addWidget(self.tag_yaw)

        control_layout = QVBoxLayout()
        control_layout.addWidget(self.control_label)
        control_layout.addWidget(self.mode_label)
        control_layout.addWidget(self.mode_combo)
        control_layout.addWidget(self.set_start_btn)
        control_layout.addWidget(self.set_goal_btn)
        control_layout.addWidget(self.add_waypoint_btn)
        control_layout.addWidget(self.execute_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addWidget(self.clear_path_btn)
        control_layout.addWidget(QLabel("경로 정보:"))
        control_layout.addWidget(self.path_info)
        
        control_frame.setLayout(control_layout)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self.robot_list)
        left_layout.addLayout(tag_form)
        left_layout.addLayout(pos_layout)
        left_layout.addLayout(yaw_layout)
        left_layout.addWidget(control_frame)
        left_layout.addWidget(self.refresh_button)
        left_layout.addStretch()

        left_widget = QWidget()
        left_widget.setLayout(left_layout)

        # ▶ 이미지 레이블
        self.image_label = ImageLabel(self)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        size_policy = QSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.image_label.setSizePolicy(size_policy)
        self.image_label.mouse_moved.connect(self.handle_mouse_move)
        self.image_label.mouse_clicked.connect(self.handle_mouse_click)

        # ▶ 전체 레이아웃
        main_layout = QHBoxLayout()
        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.image_label, stretch=1)
        self.setLayout(main_layout)

    def on_mode_changed(self, mode):
        """제어 모드 변경 시 UI 업데이트"""
        if mode == "단일 목표":
            self.set_start_btn.setEnabled(False)
            self.set_goal_btn.setEnabled(self.selected_robot_id is not None)
            self.add_waypoint_btn.setEnabled(False)
        elif mode == "경로 계획":
            self.set_start_btn.setEnabled(self.selected_robot_id is not None)
            self.set_goal_btn.setEnabled(self.selected_robot_id is not None)
            self.add_waypoint_btn.setEnabled(self.selected_robot_id is not None)
        else:  # 수동 제어
            self.set_start_btn.setEnabled(False)
            self.set_goal_btn.setEnabled(False)
            self.add_waypoint_btn.setEnabled(False)
        
        self.path_planning_mode = False
        self.current_waypoint_robot = None

    def set_start_point_mode(self):
        """시작점 설정 모드"""
        self.path_planning_mode = "start"
        self.set_start_btn.setStyleSheet("background-color: #4CAF50;")
        self.set_goal_btn.setStyleSheet("")
        self.add_waypoint_btn.setStyleSheet("")

    def set_goal_point_mode(self):
        """목표점 설정 모드"""
        self.path_planning_mode = "goal"
        self.set_start_btn.setStyleSheet("")
        self.set_goal_btn.setStyleSheet("background-color: #f44336;")
        self.add_waypoint_btn.setStyleSheet("")

    def add_waypoint_mode(self):
        """웨이포인트 추가 모드"""
        self.path_planning_mode = "waypoint"
        self.set_start_btn.setStyleSheet("")
        self.set_goal_btn.setStyleSheet("")
        self.add_waypoint_btn.setStyleSheet("background-color: #2196F3;")

    def clear_path(self):
        """경로 지우기"""
        if self.selected_robot_id is not None:
            self.waypoints[self.selected_robot_id] = []
            self.start_points.pop(self.selected_robot_id, None)
            self.goal_points.pop(self.selected_robot_id, None)
            self.update_path_info()
            print(f"🗑️ 로봇{self.selected_robot_id} 경로 지워짐")

    def execute_path(self):
        """경로 실행"""
        if self.selected_robot_id is None or self.ros_node is None:
            return
            
        mode = self.mode_combo.currentText()
        robot_id = self.selected_robot_id
        
        if mode == "단일 목표":
            if robot_id in self.goal_points:
                goal_x, goal_y = self.goal_points[robot_id]
                self.ros_node.set_robot_goal(robot_id, goal_x, goal_y)
                print(f"▶️ 로봇{robot_id} 단일 목표로 이동: ({goal_x:.3f}, {goal_y:.3f})")
        
        elif mode == "경로 계획":
            if robot_id in self.waypoints and self.waypoints[robot_id]:
                path = self.waypoints[robot_id].copy()
                if robot_id in self.goal_points:
                    path.append(self.goal_points[robot_id])
                self.ros_node.set_robot_path(robot_id, path)
                print(f"▶️ 로봇{robot_id} 경로 실행: {len(path)}개 웨이포인트")
        
        self.execute_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_robot(self):
        """로봇 정지"""
        if self.selected_robot_id is not None and self.ros_node is not None:
            self.ros_node.stop_robot(self.selected_robot_id)
            self.execute_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)

    def update_path_info(self):
        """경로 정보 업데이트"""
        if self.selected_robot_id is None:
            self.path_info.clear()
            return
            
        robot_id = self.selected_robot_id
        info_text = f"로봇 {robot_id} 경로 정보:\n"
        
        if robot_id in self.start_points:
            x, y = self.start_points[robot_id]
            info_text += f"🎯 시작점: ({x:.3f}, {y:.3f})\n"
        
        if robot_id in self.waypoints and self.waypoints[robot_id]:
            info_text += f"📍 웨이포인트 ({len(self.waypoints[robot_id])}개):\n"
            for i, (x, y) in enumerate(self.waypoints[robot_id]):
                info_text += f"  {i+1}. ({x:.3f}, {y:.3f})\n"
        
        if robot_id in self.goal_points:
            x, y = self.goal_points[robot_id]
            info_text += f"🏁 목표점: ({x:.3f}, {y:.3f})\n"
        
        self.path_info.setText(info_text)
        
        # 실행 버튼 활성화 조건
        mode = self.mode_combo.currentText()
        if mode == "단일 목표":
            self.execute_btn.setEnabled(robot_id in self.goal_points)
        elif mode == "경로 계획":
            has_waypoints = robot_id in self.waypoints and len(self.waypoints[robot_id]) > 0
            has_goal = robot_id in self.goal_points
            self.execute_btn.setEnabled(has_waypoints or has_goal)

    def handle_mouse_click(self):
        """마우스 클릭 처리"""
        if self.webcam_thread is None or self.webcam_thread.grid_corners is None:
            print("⚠️ 그리드 정보 없음")
            return

        if self.cursor_pos is None:
            print("⚠️ 커서 위치 정보 없음")
            return

        x, y = self.cursor_pos[0], self.cursor_pos[1]
        real_x, real_y = self.webcam_thread.transed_point(x, y)

        if not (0 <= real_x <= REAL_MAX_WIDTH and 0 <= real_y <= REAL_MAX_HEIGHT):
            print("⚠️ 클릭 위치가 그리드 범위를 벗어남")
            return

        print(f"🖱️ 클릭 위치: 이미지({x}, {y}) → 실제({real_x:.3f}, {real_y:.3f})")

        if self.selected_robot_id is None:
            print("⚠️ 로봇을 먼저 선택하세요")
            return

        robot_id = self.selected_robot_id
        
        # 제어 모드에 따른 처리
        if self.path_planning_mode == "start":
            self.start_points[robot_id] = (real_x, real_y)
            print(f"🎯 로봇{robot_id} 시작점 설정: ({real_x:.3f}, {real_y:.3f})")
            self.path_planning_mode = False
            self.set_start_btn.setStyleSheet("")
            
        elif self.path_planning_mode == "goal":
            self.goal_points[robot_id] = (real_x, real_y)
            print(f"🏁 로봇{robot_id} 목표점 설정: ({real_x:.3f}, {real_y:.3f})")
            self.path_planning_mode = False
            self.set_goal_btn.setStyleSheet("")
            
        elif self.path_planning_mode == "waypoint":
            if robot_id not in self.waypoints:
                self.waypoints[robot_id] = []
            self.waypoints[robot_id].append((real_x, real_y))
            print(f"📍 로봇{robot_id} 웨이포인트 추가: ({real_x:.3f}, {real_y:.3f})")
            # 웨이포인트 모드는 연속으로 추가할 수 있도록 유지
            
        else:
            # 기본 모드: 즉시 목표점으로 이동
            mode = self.mode_combo.currentText()
            if mode == "단일 목표":
                self.goal_points[robot_id] = (real_x, real_y)
                if self.ros_node:
                    self.ros_node.set_robot_goal(robot_id, real_x, real_y)
                    print(f"🎯 로봇{robot_id} 즉시 이동: ({real_x:.3f}, {real_y:.3f})")

        self.update_path_info()

    def handle_robot_select(self, item):
        """로봇 선택 처리"""
        text = item.text()
        if "로봇-" in text:
            robot_id = int(text.split("-")[1])
            self.selected_robot_id = robot_id
            
            # 기존 스타일 초기화
            for i in range(self.robot_list.count()):
                self.robot_list.item(i).setBackground(Qt.GlobalColor.white)
            
            # 선택된 항목 하이라이트
            item.setBackground(Qt.GlobalColor.cyan)

            
            # 태그 정보 업데이트
            tag_id = self.robot_tag_map.get(robot_id, "")
            self.tag_edit.setText(str(tag_id))
            
            if self.webcam_thread:
                poses = self.webcam_thread.get_tag_pose(tag_id)
                if poses:
                    pose = poses[0]
                    self.tag_x.setText(f"{pose[0]:.1f}")
                    self.tag_y.setText(f"{pose[1]:.1f}")
                    self.tag_yaw.setText(f"{pose[2]:.3f}")
            
            # 버튼 상태 업데이트
            mode = self.mode_combo.currentText()
            self.on_mode_changed(mode)
            self.update_path_info()
            
            print(f"🤖 로봇 {robot_id} 선택됨")

    def save_tag_id(self):
        """태그 ID 저장"""
        if self.selected_robot_id is not None:
            new_id = self.tag_edit.text()
            if new_id.isdigit():
                self.robot_tag_map[self.selected_robot_id] = int(new_id)
                print(f"💾 로봇{self.selected_robot_id} → 태그 ID: {new_id} 저장됨")

    def handle_mouse_move(self, pos: QPoint):
        """마우스 이동 처리"""
        if self.current_frame is None:
            return

        # QLabel 크기
        label_w = self.image_label.width()
        label_h = self.image_label.height()

        # 이미지 크기
        img_h, img_w, _ = self.current_frame.shape

        # 이미지와 라벨 사이즈의 비율 계산
        scale_w = label_w / img_w
        scale_h = label_h / img_h

        # KeepAspectRatio에 따라 실제 스케일을 선택
        scale = min(scale_w, scale_h)

        # 이미지가 QLabel 내에서 차지하는 실제 크기
        displayed_w = int(img_w * scale)
        displayed_h = int(img_h * scale)

        # QLabel 중앙 정렬 오프셋 계산 (여백)
        offset_x = (label_w - displayed_w) // 2
        offset_y = (label_h - displayed_h) // 2

        # 마우스 위치 (QLabel 기준) → 이미지 내 좌표로 역변환
        x_in_label = pos.x() - offset_x
        y_in_label = pos.y() - offset_y

        if x_in_label < 0 or y_in_label < 0 or x_in_label >= displayed_w or y_in_label >= displayed_h:
            self.cursor_pos = None  # 바깥 영역일 경우
        else:
            # 정규화된 이미지 좌표 계산
            real_x = int(x_in_label / scale)
            real_y = int(y_in_label / scale)
            self.cursor_pos = (real_x, real_y)

        self.refresh_image()

    def handle_refresh_click(self):
        """그리드 리셋"""
        if self.webcam_thread is not None and self.current_frame is not None:
            self.webcam_thread.redraw_grid_corners(self.current_frame)
            self.refresh_image()

    def set_ros_node(self, node):
        """ROS 노드 설정"""
        self.ros_node = node

    def set_webcam_thread(self, thread):
        """웹캠 스레드 설정"""
        self.webcam_thread = thread

    def update_image(self, frame):
        """이미지 업데이트"""
        self.current_frame = frame
        self.refresh_image()

    def resizeEvent(self, event):
        """창 크기 변경 시"""
        self.refresh_image()
        return super().resizeEvent(event)

    def refresh_image(self):
        """이미지 갱신"""
        if self.current_frame is None:
            return

        rgb_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)

        # 커서 위치 표시
        if self.cursor_pos is not None:
            x, y = self.cursor_pos
            if self.webcam_thread and self.webcam_thread.grid_corners:
                real_x, real_y = self.webcam_thread.transed_point(x, y)
                if 0 <= real_x <= REAL_MAX_WIDTH and 0 <= real_y <= REAL_MAX_HEIGHT:
                    cv2.putText(
                        rgb_image,
                        f"({real_x:.3f}, {real_y:.3f})",
                        (x + 10, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )
            cv2.circle(rgb_image, (x, y), 5, (255, 255, 255), 2)

        # 경로 시각화
        self.draw_paths(rgb_image)

        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        pixmap = QPixmap.fromImage(q_image)
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.image_label.setPixmap(scaled)

    def draw_paths(self, image):
        """경로 시각화"""
        if not self.webcam_thread or not self.webcam_thread.grid_corners:
            return

        for robot_id in self.robot_tag_map.keys():
            # 시작점
            if robot_id in self.start_points:
                real_x, real_y = self.start_points[robot_id]
                img_pt = self.real_to_image_point(real_x, real_y)
                if img_pt:
                    cv2.circle(image, img_pt, 10, (0, 255, 0), 3)  # 녹색 원
                    cv2.putText(image, f"S{robot_id}", (img_pt[0]-10, img_pt[1]-15), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 웨이포인트들
            if robot_id in self.waypoints:
                prev_point = None
                for i, (real_x, real_y) in enumerate(self.waypoints[robot_id]):
                    img_pt = self.real_to_image_point(real_x, real_y)
                    if img_pt:
                        cv2.circle(image, img_pt, 8, (255, 0, 0), 2)  # 파란색 원
                        cv2.putText(image, f"{i+1}", (img_pt[0]-5, img_pt[1]+5), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        
                        # 이전 점과 연결
                        if prev_point:
                            cv2.line(image, prev_point, img_pt, (255, 0, 0), 2)
                        prev_point = img_pt

            # 목표점
            if robot_id in self.goal_points:
                real_x, real_y = self.goal_points[robot_id]
                img_pt = self.real_to_image_point(real_x, real_y)
                if img_pt:
                    cv2.circle(image, img_pt, 12, (0, 0, 255), 3)  # 빨간색 원
                    cv2.putText(image, f"G{robot_id}", (img_pt[0]-10, img_pt[1]-15), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # 마지막 웨이포인트와 연결
                    if robot_id in self.waypoints and self.waypoints[robot_id]:
                        last_waypoint = self.waypoints[robot_id][-1]
                        last_img_pt = self.real_to_image_point(last_waypoint[0], last_waypoint[1])
                        if last_img_pt:
                            cv2.line(image, last_img_pt, img_pt, (255, 0, 0), 2)

    def real_to_image_point(self, real_x, real_y):
        """실제 좌표를 이미지 좌표로 변환"""
        if not self.webcam_thread or not self.webcam_thread.grid_corners:
            return None
            
        try:
            # 실제 좌표를 정규화된 좌표로 변환
            u = real_x / REAL_MAX_WIDTH
            v = (REAL_MAX_HEIGHT - real_y) / REAL_MAX_HEIGHT
            
            # 그리드 코너 정보
            grid_corners = np.array(self.webcam_thread.grid_corners, dtype=np.float32)
            unit_square = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=np.float32)
            
            # 역변환 매트릭스 계산
            M = cv2.getPerspectiveTransform(unit_square, grid_corners)
            
            # 정규화된 좌표를 이미지 좌표로 변환
            norm_point = np.array([[[u, v]]], dtype=np.float32)
            img_point = cv2.perspectiveTransform(norm_point, M)
            
            x, y = int(img_point[0][0][0]), int(img_point[0][0][1])
            return (x, y)
            
        except Exception as e:
            print(f"⚠️ 좌표 변환 실패: {e}")
            return None


# === 메인 함수 ===
def main():
    # 로봇 ID와 에이프릴태그 ID 매핑
    robot_tag_map = {
        1: 2,  # 로봇 1 → 태그 2
        2: 4,  # 로봇 2 → 태그 4
        # 필요시 추가
    }

    app = QApplication(sys.argv)
    image_window = ImageWindow(robot_tag_map=robot_tag_map)
    image_window.show()

    # 웹캠 스레드 시작
    webcam_thread = WebcamThread(
        tag_id=TAG_ID,
        camera_index=CAMERA_INDEX,
        visualize=True,
        frame_callback=image_window.update_image,
    )
    webcam_thread.start()
    image_window.set_webcam_thread(webcam_thread)

    # ROS 초기화
    rclpy.init(args=None)
    robot_ids = list(robot_tag_map.keys())

    # pose 함수 매핑 생성
    def make_pose_func(tag_id):
        return lambda: webcam_thread.get_transed_pose_for_tag(tag_id)

    get_pose_func_map = {rid: make_pose_func(robot_tag_map[rid]) for rid in robot_ids}

    # ROS 노드 생성
    multi_publisher_node = PoseMultiPublisher(
        robot_ids, get_pose_func_map, publish_period=0.1
    )
    image_window.set_ros_node(multi_publisher_node)

    # ROS 스핀 스레드
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(multi_publisher_node), daemon=True
    )
    ros_thread.start()

    print("🚀 AprilTag Robot Controller 시작!")
    print("📋 사용법:")
    print("  1. 로봇을 선택하세요")
    print("  2. 제어 모드를 선택하세요")
    print("  3. 시작점/목표점을 클릭으로 설정하세요")
    print("  4. '실행' 버튼을 눌러 로봇을 움직이세요")

    # Qt 메인 루프 실행
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("🛑 종료 요청 받음")
    finally:
        multi_publisher_node.destroy_node()
        rclpy.shutdown()
        webcam_thread.stop()
        webcam_thread.join()
        print("✅ 프로그램 종료")

if __name__ == '__main__':
    main()