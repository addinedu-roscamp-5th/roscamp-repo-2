#!/usr/bin/env python3
"""
다중로봇 카메라 모니터링 시스템
AprilTag를 이용한 실시간 로봇 위치 추적 및 ROS 토픽 발행
"""

import threading
import time
import json
import sys
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple, Callable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import cv2
import numpy as np
from pupil_apriltags import Detector
from math import atan2, sin, cos, pi, sqrt

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, 
    QVBoxLayout, QHBoxLayout, QGridLayout, QListWidget, QListWidgetItem, 
    QLineEdit, QSpinBox, QDoubleSpinBox, QCheckBox, QComboBox,
    QTextEdit, QTabWidget, QGroupBox, QSlider, QProgressBar,
    QMessageBox, QFileDialog, QStatusBar
)
from PySide6.QtGui import QImage, QPixmap, QFont, QColor, QPalette
from PySide6.QtCore import Qt, Signal, QPoint, QTimer


@dataclass
class RobotConfig:
    """로봇 설정 데이터클래스"""
    robot_id: int
    tag_id: int
    name: str = ""
    color: Tuple[int, int, int] = (0, 255, 0)
    active: bool = True
    last_seen: float = 0.0
    
    def __post_init__(self):
        if not self.name:
            self.name = f"Robot-{self.robot_id}"


@dataclass
class CameraConfig:
    """카메라 설정 데이터클래스"""
    camera_index: int = 0
    width: int = 1920
    height: int = 1080
    fps: int = 30
    
    # 격자 설정
    grid_rows: int = 12
    grid_cols: int = 24
    real_width: float = 1.91  # 미터
    real_height: float = 0.91  # 미터
    
    # 칼리브레이션 포인트 (카메라 좌표계)
    top_left: Tuple[int, int] = (0, 0)
    top_right: Tuple[int, int] = (1919, 0)
    bottom_left: Tuple[int, int] = (0, 1079)
    bottom_right: Tuple[int, int] = (1919, 1079)


class ImageLabel(QLabel):
    """마우스 이벤트를 처리하는 이미지 라벨"""
    mouse_clicked = Signal(QPoint)
    mouse_moved = Signal(QPoint)
    mouse_right_clicked = Signal(QPoint)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setMinimumSize(640, 480)

    def mouseMoveEvent(self, event):
        self.mouse_moved.emit(event.position().toPoint())

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.mouse_clicked.emit(event.position().toPoint())
        elif event.button() == Qt.MouseButton.RightButton:
            self.mouse_right_clicked.emit(event.position().toPoint())


class AprilTagDetector(threading.Thread):
    """AprilTag 검출 스레드"""
    
    def __init__(self, camera_config: CameraConfig, robot_configs: Dict[int, RobotConfig]):
        super().__init__(daemon=True)
        self.camera_config = camera_config
        self.robot_configs = robot_configs
        self.detector = Detector(families="tag36h11")
        
        # 동기화용 락
        self.result_lock = threading.Lock()
        self.running = False
        
        # 검출 결과
        self.all_detections = []
        self.current_frame = None
        self.grid_corners = None
        self.transformation_matrix = None
        
        # 콜백
        self.frame_callback = None
        
        # 통계
        self.fps = 0
        self.detection_count = 0
        
    def set_frame_callback(self, callback: Callable):
        self.frame_callback = callback
        
    def update_grid_corners(self, corners: List[Tuple[int, int]]):
        """격자 모서리 업데이트"""
        if len(corners) == 4:
            self.grid_corners = np.array(corners, dtype=np.float32)
            self._update_transformation_matrix()
            
    def _update_transformation_matrix(self):
        """원근 변환 매트릭스 업데이트"""
        if self.grid_corners is not None:
            unit_square = np.array([
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
            ], dtype=np.float32)
            
            try:
                self.transformation_matrix = cv2.getPerspectiveTransform(
                    self.grid_corners, unit_square
                )
            except cv2.error as e:
                print(f"⚠️ 원근 변환 매트릭스 계산 실패: {e}")
                self.transformation_matrix = None
    
    def image_to_real_coords(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """이미지 좌표를 실제 좌표로 변환"""
        if self.transformation_matrix is None:
            return None
            
        img_point = np.array([[[x, y]]], dtype=np.float32)
        try:
            normalized = cv2.perspectiveTransform(img_point, self.transformation_matrix)
            u, v = normalized[0][0]
            
            real_x = u * self.camera_config.real_width
            real_y = self.camera_config.real_height - v * self.camera_config.real_height
            
            # 유효 범위 체크
            if 0 <= real_x <= self.camera_config.real_width and 0 <= real_y <= self.camera_config.real_height:
                return real_x, real_y
        except cv2.error:
            pass
        return None
    
    def get_robot_pose(self, robot_id: int) -> Optional[Tuple[float, float, float]]:
        """특정 로봇의 포즈 반환 (real coordinates)"""
        config = self.robot_configs.get(robot_id)
        if not config or not config.active:
            return None
            
        with self.result_lock:
            for detection in self.all_detections:
                if detection['tag_id'] == config.tag_id:
                    x, y, yaw = detection['pose']
                    real_coords = self.image_to_real_coords(x, y)
                    if real_coords:
                        real_x, real_y = real_coords
                        return real_x, real_y, yaw
        return None
    
    def get_all_detections(self) -> List[Dict]:
        """모든 검출 결과 반환"""
        with self.result_lock:
            return list(self.all_detections)
    
    def run(self):
        """메인 검출 루프"""
        cap = cv2.VideoCapture(self.camera_config.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.height)
        cap.set(cv2.CAP_PROP_FPS, self.camera_config.fps)
        
        if not cap.isOpened():
            print("❌ 카메라 열기 실패")
            return
            
        print(f"🎥 카메라 시작: {self.camera_config.width}x{self.camera_config.height}")
        self.running = True
        
        frame_count = 0
        start_time = time.time()
        
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
                
            # FPS 계산
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                self.fps = 30 / elapsed if elapsed > 0 else 0
                start_time = time.time()
            
            # AprilTag 검출
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(gray, estimate_tag_pose=False)
            
            current_detections = []
            current_time = time.time()
            
            for tag in tags:
                center_x, center_y = int(tag.center[0]), int(tag.center[1])
                
                # 방향 계산
                pt0, pt1 = tag.corners[0], tag.corners[1]
                dx = pt1[0] - pt0[0]
                dy = pt1[1] - pt0[1]
                yaw = atan2(dy, dx)
                
                detection = {
                    'tag_id': tag.tag_id,
                    'pose': (center_x, center_y, -yaw),
                    'corners': [(int(p[0]), int(p[1])) for p in tag.corners],
                    'timestamp': current_time
                }
                current_detections.append(detection)
                
                # 로봇 설정 업데이트 (last_seen)
                for robot_config in self.robot_configs.values():
                    if robot_config.tag_id == tag.tag_id:
                        robot_config.last_seen = current_time
            
            with self.result_lock:
                self.all_detections = current_detections
                self.detection_count = len(current_detections)
                self.current_frame = frame.copy()
            
            # 시각화
            self._draw_detections(frame)
            
            if self.frame_callback:
                self.frame_callback(frame)
        
        cap.release()
        print("🛑 카메라 스레드 종료")
    
    def _draw_detections(self, frame):
        """검출 결과 시각화"""
        for detection in self.all_detections:
            tag_id = detection['tag_id']
            center_x, center_y, yaw = detection['pose']
            corners = detection['corners']
            
            # 로봇 설정에서 색상 가져오기
            color = (0, 255, 0)  # 기본 초록색
            for robot_config in self.robot_configs.values():
                if robot_config.tag_id == tag_id:
                    color = robot_config.color
                    break
            
            # 태그 외곽선
            corners_array = np.array(corners, dtype=np.int32)
            cv2.polylines(frame, [corners_array], isClosed=True, color=color, thickness=2)
            
            # 방향 화살표
            arrow_len = 40
            end_x = int(center_x + arrow_len * cos(-yaw))
            end_y = int(center_y + arrow_len * sin(-yaw))
            cv2.arrowedLine(frame, (center_x, center_y), (end_x, end_y), 
                          (0, 0, 255), 3, tipLength=0.3)
            
            # 태그 ID 표시
            cv2.putText(frame, f"ID:{tag_id}", (center_x-20, center_y-15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 실제 좌표 표시
            real_coords = self.image_to_real_coords(center_x, center_y)
            if real_coords:
                real_x, real_y = real_coords
                coord_text = f"({real_x:.2f}, {real_y:.2f})"
                cv2.putText(frame, coord_text, (center_x-30, center_y+30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 격자 그리기
        self._draw_grid(frame)
        
        # 정보 표시
        info_text = f"FPS: {self.fps:.1f} | Tags: {self.detection_count}"
        cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (0, 255, 255), 2)
    
    def _draw_grid(self, frame):
        """격자 그리기"""
        if self.grid_corners is None:
            return
            
        rows = self.camera_config.grid_rows + 1
        cols = self.camera_config.grid_cols + 1
        
        # 격자 점 생성
        for i in range(rows):
            for j in range(cols):
                # 정규화된 좌표
                u = j / (cols - 1)
                v = i / (rows - 1)
                
                # 이미지 좌표로 변환
                point = np.array([[[u, v]]], dtype=np.float32)
                try:
                    inv_matrix = cv2.getPerspectiveTransform(
                        np.array([[0,0],[1,0],[0,1],[1,1]], dtype=np.float32),
                        self.grid_corners
                    )
                    img_point = cv2.perspectiveTransform(point, inv_matrix)
                    x, y = int(img_point[0][0][0]), int(img_point[0][0][1])
                    
                    # 격자 점 그리기
                    cv2.circle(frame, (x, y), 2, (255, 255, 0), -1)
                    
                    # 격자 선 그리기
                    if j > 0:  # 가로선
                        prev_u = (j-1) / (cols - 1)
                        prev_point = np.array([[[prev_u, v]]], dtype=np.float32)
                        prev_img = cv2.perspectiveTransform(prev_point, inv_matrix)
                        prev_x, prev_y = int(prev_img[0][0][0]), int(prev_img[0][0][1])
                        cv2.line(frame, (prev_x, prev_y), (x, y), (255, 255, 0), 1)
                    
                    if i > 0:  # 세로선
                        prev_v = (i-1) / (rows - 1)
                        prev_point = np.array([[[u, prev_v]]], dtype=np.float32)
                        prev_img = cv2.perspectiveTransform(prev_point, inv_matrix)
                        prev_x, prev_y = int(prev_img[0][0][0]), int(prev_img[0][0][1])
                        cv2.line(frame, (prev_x, prev_y), (x, y), (255, 255, 0), 1)
                        
                except cv2.error:
                    pass
    
    def stop(self):
        self.running = False


class ROSPublisher(Node):
    """ROS 토픽 발행 노드"""
    
    def __init__(self, robot_configs: Dict[int, RobotConfig], detector: AprilTagDetector):
        super().__init__('multi_robot_monitor')
        self.robot_configs = robot_configs
        self.detector = detector
        
        # Publishers
        self.pose_publishers = {}
        self.marker_pub = self.create_publisher(MarkerArray, '/robot_markers', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/camera_map', 10)
        
        # 각 로봇별 pose publisher 생성
        for robot_id in robot_configs:
            topic = f'/robot_{robot_id}/camera_pose'
            self.pose_publishers[robot_id] = self.create_publisher(
                PoseStamped, topic, 10
            )
        
        # 타이머 설정
        self.create_timer(0.1, self.publish_poses)  # 10Hz
        self.create_timer(0.5, self.publish_markers)  # 2Hz
        self.create_timer(2.0, self.publish_map)  # 0.5Hz
        
        self.get_logger().info(f"다중로봇 모니터 시작 - {len(robot_configs)}개 로봇")
    
    def publish_poses(self):
        """로봇 포즈 발행"""
        current_time = self.get_clock().now()
        
        for robot_id, config in self.robot_configs.items():
            if not config.active:
                continue
                
            pose = self.detector.get_robot_pose(robot_id)
            if pose is None:
                continue
                
            x, y, yaw = pose
            
            msg = PoseStamped()
            msg.header.stamp = current_time.to_msg()
            msg.header.frame_id = 'camera_map'
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = 0.0
            msg.pose.orientation.z = sin(yaw / 2.0)
            msg.pose.orientation.w = cos(yaw / 2.0)
            
            self.pose_publishers[robot_id].publish(msg)
    
    def publish_markers(self):
        """로봇 마커 발행 (RViz 시각화용)"""
        marker_array = MarkerArray()
        current_time = self.get_clock().now()
        
        for robot_id, config in self.robot_configs.items():
            if not config.active:
                continue
                
            pose = self.detector.get_robot_pose(robot_id)
            if pose is None:
                continue
                
            x, y, yaw = pose
            
            # 로봇 마커
            marker = Marker()
            marker.header.frame_id = 'camera_map'
            marker.header.stamp = current_time.to_msg()
            marker.id = robot_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.1
            marker.pose.orientation.z = sin(yaw / 2.0)
            marker.pose.orientation.w = cos(yaw / 2.0)
            
            marker.scale.x = 0.15
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # 로봇별 색상
            r, g, b = config.color
            marker.color.r = r / 255.0
            marker.color.g = g / 255.0
            marker.color.b = b / 255.0
            marker.color.a = 1.0
            
            # 텍스트 마커
            text_marker = Marker()
            text_marker.header.frame_id = 'camera_map'
            text_marker.header.stamp = current_time.to_msg()
            text_marker.id = robot_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = 0.2
            
            text_marker.text = config.name
            text_marker.scale.z = 0.1
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.extend([marker, text_marker])
        
        self.marker_pub.publish(marker_array)
    
    def publish_map(self):
        """카메라 영역 맵 발행"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'camera_map'
        
        # 맵 메타데이터
        map_msg.info.resolution = 0.05  # 5cm per pixel
        map_msg.info.width = int(self.detector.camera_config.real_width / map_msg.info.resolution)
        map_msg.info.height = int(self.detector.camera_config.real_height / map_msg.info.resolution)
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # 맵 데이터 (모두 자유 공간으로 설정)
        map_data = [0] * (map_msg.info.width * map_msg.info.height)
        map_msg.data = map_data
        
        self.map_pub.publish(map_msg)


class RobotConfigWidget(QWidget):
    """로봇 설정 위젯"""
    
    def __init__(self, robot_config: RobotConfig, parent=None):
        super().__init__(parent)
        self.robot_config = robot_config
        self.setup_ui()
    
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # 로봇 이름
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("이름:"))
        self.name_edit = QLineEdit(self.robot_config.name)
        self.name_edit.textChanged.connect(self.update_name)
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # 태그 ID
        tag_layout = QHBoxLayout()
        tag_layout.addWidget(QLabel("태그 ID:"))
        self.tag_spin = QSpinBox()
        self.tag_spin.setRange(0, 999)
        self.tag_spin.setValue(self.robot_config.tag_id)
        self.tag_spin.valueChanged.connect(self.update_tag_id)
        tag_layout.addWidget(self.tag_spin)
        layout.addLayout(tag_layout)
        
        # 활성화
        self.active_check = QCheckBox("활성화")
        self.active_check.setChecked(self.robot_config.active)
        self.active_check.toggled.connect(self.update_active)
        layout.addWidget(self.active_check)
        
        # 색상 (간단한 선택)
        color_layout = QHBoxLayout()
        color_layout.addWidget(QLabel("색상:"))
        self.color_combo = QComboBox()
        colors = [
            ("초록", (0, 255, 0)),
            ("빨강", (255, 0, 0)),
            ("파랑", (0, 0, 255)),
            ("노랑", (255, 255, 0)),
            ("자홍", (255, 0, 255)),
            ("청록", (0, 255, 255)),
        ]
        for name, color in colors:
            self.color_combo.addItem(name, color)
        color_layout.addWidget(self.color_combo)
        self.color_combo.currentTextChanged.connect(self.update_color)
        layout.addLayout(color_layout)
        
        self.setLayout(layout)
    
    def update_name(self, text):
        self.robot_config.name = text
    
    def update_tag_id(self, value):
        self.robot_config.tag_id = value
    
    def update_active(self, checked):
        self.robot_config.active = checked
    
    def update_color(self, text):
        current_index = self.color_combo.currentIndex()
        if current_index >= 0:
            color = self.color_combo.itemData(current_index)
            if color:
                self.robot_config.color = color


class MonitorMainWindow(QMainWindow):
    """메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("다중로봇 카메라 모니터링 시스템")
        self.setGeometry(100, 100, 1400, 800)
        
        # 설정 초기화
        self.camera_config = CameraConfig()
        self.robot_configs = {
            1: RobotConfig(1, 2, "로봇-1", (0, 255, 0)),
            2: RobotConfig(2, 4, "로봇-2", (255, 0, 0)),
            3: RobotConfig(3, 6, "로봇-3", (0, 0, 255)),
        }
        
        # 검출기 및 ROS 노드
        self.detector = None
        self.ros_node = None
        self.ros_thread = None
        
        # UI 설정
        self.setup_ui()
        self.setup_status_bar()
        
        # 타이머 설정
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(100)  # 10Hz UI 업데이트
        
        # 마우스 상태
        self.mouse_pos = None
        self.calibration_mode = False
        self.calibration_points = []
        
    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout()
        
        # 왼쪽 패널 (설정)
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel)
        
        # 중앙 패널 (이미지)
        center_panel = self.create_center_panel()
        main_layout.addWidget(center_panel, stretch=2)
        
        # 오른쪽 패널 (정보)
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel)
        
        central_widget.setLayout(main_layout)
    
    def create_left_panel(self):
        """왼쪽 설정 패널 생성"""
        panel = QWidget()
        panel.setFixedWidth(300)
        layout = QVBoxLayout()
        
        # 카메라 제어
        camera_group = QGroupBox("카메라 제어")
        camera_layout = QVBoxLayout()
        
        self.start_button = QPushButton("🎥 시작")
        self.start_button.clicked.connect(self.start_monitoring)
        camera_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("⏹️ 정지")
        self.stop_button.clicked.connect(self.stop_monitoring)
        self.stop_button.setEnabled(False)
        camera_layout.addWidget(self.stop_button)
        
        camera_group.setLayout(camera_layout)
        layout.addWidget(camera_group)
        
        # 칼리브레이션
        calib_group = QGroupBox("칼리브레이션")
        calib_layout = QVBoxLayout()
        
        self.calib_button = QPushButton("🎯 칼리브레이션 시작")
        self.calib_button.clicked.connect(self.start_calibration)
        calib_layout.addWidget(self.calib_button)
        
        self.reset_calib_button = QPushButton("🔄 리셋")
        self.reset_calib_button.clicked.connect(self.reset_calibration)
        calib_layout.addWidget(self.reset_calib_button)
        
        calib_group.setLayout(calib_layout)
        layout.addWidget(calib_group)
        
        # 로봇 설정
        robot_group = QGroupBox("로봇 설정")
        robot_layout = QVBoxLayout()
        
        self.robot_tabs = QTabWidget()
        for robot_id, config in self.robot_configs.items():
            widget = RobotConfigWidget(config)
            self.robot_tabs.addTab(widget, f"로봇-{robot_id}")
        
        robot_layout.addWidget(self.robot_tabs)
        robot_group.setLayout(robot_layout)
        layout.addWidget(robot_group)
        
        # 설정 저장/로드
        file_group = QGroupBox("설정 파일")
        file_layout = QVBoxLayout()
        
        save_button = QPushButton("💾 설정 저장")
        save_button.clicked.connect(self.save_config)
        file_layout.addWidget(save_button)
        
        load_button = QPushButton("📁 설정 로드")
        load_button.clicked.connect(self.load_config)
        file_layout.addWidget(load_button)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        layout.addStretch()
        panel.setLayout(layout)
        return panel
    
    def create_center_panel(self):
        """중앙 이미지 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # 이미지 라벨
        self.image_label = ImageLabel()
        self.image_label.setStyleSheet("border: 1px solid gray;")
        self.image_label.mouse_clicked.connect(self.handle_mouse_click)
        self.image_label.mouse_moved.connect(self.handle_mouse_move)
        self.image_label.mouse_right_clicked.connect(self.handle_right_click)
        
        layout.addWidget(self.image_label)
        
        # 하단 컨트롤
        control_layout = QHBoxLayout()
        
        self.coord_label = QLabel("마우스: (0, 0)")
        control_layout.addWidget(self.coord_label)
        
        control_layout.addStretch()
        
        self.fps_label = QLabel("FPS: 0.0")
        control_layout.addWidget(self.fps_label)
        
        self.detection_label = QLabel("검출: 0")
        control_layout.addWidget(self.detection_label)
        
        layout.addLayout(control_layout)
        
        panel.setLayout(layout)
        return panel
    
    def create_right_panel(self):
        """오른쪽 정보 패널 생성"""
        panel = QWidget()
        panel.setFixedWidth(300)
        layout = QVBoxLayout()
        
        # 로봇 상태
        status_group = QGroupBox("로봇 상태")
        status_layout = QVBoxLayout()
        
        self.robot_status_list = QListWidget()
        status_layout.addWidget(self.robot_status_list)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 검출 로그
        log_group = QGroupBox("검출 로그")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(200)
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        clear_log_button = QPushButton("🗑️ 로그 지우기")
        clear_log_button.clicked.connect(self.clear_log)
        log_layout.addWidget(clear_log_button)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        # 통계
        stats_group = QGroupBox("통계")
        stats_layout = QVBoxLayout()
        
        self.total_detections_label = QLabel("총 검출: 0")
        stats_layout.addWidget(self.total_detections_label)
        
        self.uptime_label = QLabel("가동 시간: 00:00:00")
        stats_layout.addWidget(self.uptime_label)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        layout.addStretch()
        panel.setLayout(layout)
        return panel
    
    def setup_status_bar(self):
        """상태바 설정"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        self.status_bar.showMessage("시스템 대기 중...")
        
        # ROS 연결 상태
        self.ros_status_label = QLabel("ROS: 연결 안됨")
        self.status_bar.addPermanentWidget(self.ros_status_label)
        
        # 카메라 상태
        self.camera_status_label = QLabel("카메라: 대기")
        self.status_bar.addPermanentWidget(self.camera_status_label)
    
    def start_monitoring(self):
        """모니터링 시작"""
        try:
            # ROS 초기화
            if not rclpy.ok():
                rclpy.init()
            
            # 검출기 시작
            self.detector = AprilTagDetector(self.camera_config, self.robot_configs)
            self.detector.set_frame_callback(self.update_image)
            self.detector.start()
            
            # ROS 노드 시작
            self.ros_node = ROSPublisher(self.robot_configs, self.detector)
            self.ros_thread = threading.Thread(
                target=lambda: rclpy.spin(self.ros_node),
                daemon=True
            )
            self.ros_thread.start()
            
            # UI 업데이트
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.camera_status_label.setText("카메라: 실행 중")
            self.ros_status_label.setText("ROS: 연결됨")
            self.status_bar.showMessage("모니터링 실행 중...")
            
            self.start_time = time.time()
            self.total_detections = 0
            
            self.log_message("모니터링 시작됨")
            
        except Exception as e:
            QMessageBox.critical(self, "오류", f"모니터링 시작 실패:\n{e}")
    
    def stop_monitoring(self):
        """모니터링 정지"""
        try:
            # 검출기 정지
            if self.detector:
                self.detector.stop()
                self.detector.join()
                self.detector = None
            
            # ROS 노드 정지
            if self.ros_node:
                self.ros_node.destroy_node()
                self.ros_node = None
            
            # UI 업데이트
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.camera_status_label.setText("카메라: 정지됨")
            self.ros_status_label.setText("ROS: 연결 안됨")
            self.status_bar.showMessage("모니터링 정지됨")
            
            # 이미지 클리어
            self.image_label.clear()
            self.image_label.setText("카메라 대기 중...")
            
            self.log_message("모니터링 정지됨")
            
        except Exception as e:
            QMessageBox.critical(self, "오류", f"모니터링 정지 실패:\n{e}")
    
    def start_calibration(self):
        """칼리브레이션 시작"""
        if not self.detector:
            QMessageBox.warning(self, "경고", "먼저 카메라를 시작하세요.")
            return
        
        self.calibration_mode = True
        self.calibration_points = []
        self.calib_button.setText("칼리브레이션 중... (4점 클릭)")
        self.calib_button.setEnabled(False)
        self.status_bar.showMessage("칼리브레이션: 좌상단부터 시계방향으로 4개 점을 클릭하세요.")
        self.log_message("칼리브레이션 시작")
    
    def reset_calibration(self):
        """칼리브레이션 리셋"""
        if self.detector:
            # 기본 코너로 리셋
            default_corners = [
                (0, 0),
                (self.camera_config.width-1, 0),
                (0, self.camera_config.height-1),
                (self.camera_config.width-1, self.camera_config.height-1)
            ]
            self.detector.update_grid_corners(default_corners)
        
        self.calibration_mode = False
        self.calibration_points = []
        self.calib_button.setText("🎯 칼리브레이션 시작")
        self.calib_button.setEnabled(True)
        self.status_bar.showMessage("칼리브레이션 리셋됨")
        self.log_message("칼리브레이션 리셋")
    
    def handle_mouse_click(self, pos: QPoint):
        """마우스 클릭 처리"""
        if not self.detector or not self.detector.current_frame is not None:
            return
        
        # 이미지 좌표 변환
        img_pos = self.convert_label_to_image_coords(pos)
        if img_pos is None:
            return
        
        x, y = img_pos
        
        if self.calibration_mode:
            self.calibration_points.append((x, y))
            self.log_message(f"칼리브레이션 점 {len(self.calibration_points)}: ({x}, {y})")
            
            if len(self.calibration_points) == 4:
                # 칼리브레이션 완료
                self.detector.update_grid_corners(self.calibration_points)
                self.calibration_mode = False
                self.calib_button.setText("🎯 칼리브레이션 시작")
                self.calib_button.setEnabled(True)
                self.status_bar.showMessage("칼리브레이션 완료")
                self.log_message("칼리브레이션 완료")
            else:
                remaining = 4 - len(self.calibration_points)
                self.status_bar.showMessage(f"칼리브레이션: {remaining}개 점이 더 필요합니다.")
        else:
            # 일반 클릭 - 실제 좌표 표시
            if self.detector:
                real_coords = self.detector.image_to_real_coords(x, y)
                if real_coords:
                    real_x, real_y = real_coords
                    self.log_message(f"클릭 위치: 이미지({x}, {y}) → 실제({real_x:.3f}, {real_y:.3f})")
    
    def handle_right_click(self, pos: QPoint):
        """우클릭 처리 - 로봇 추가/제거"""
        if not self.detector:
            return
        
        img_pos = self.convert_label_to_image_coords(pos)
        if img_pos is None:
            return
        
        # 가장 가까운 태그 찾기
        detections = self.detector.get_all_detections()
        min_dist = float('inf')
        closest_tag = None
        
        for detection in detections:
            tag_x, tag_y, _ = detection['pose']
            dist = sqrt((img_pos[0] - tag_x)**2 + (img_pos[1] - tag_y)**2)
            if dist < min_dist and dist < 50:  # 50픽셀 이내
                min_dist = dist
                closest_tag = detection
        
        if closest_tag:
            tag_id = closest_tag['tag_id']
            self.log_message(f"태그 {tag_id} 선택됨")
    
    def handle_mouse_move(self, pos: QPoint):
        """마우스 이동 처리"""
        if not self.detector or self.detector.current_frame is None:
            return
        
        img_pos = self.convert_label_to_image_coords(pos)
        if img_pos is None:
            self.coord_label.setText("마우스: 범위 밖")
            return
        
        x, y = img_pos
        coord_text = f"마우스: ({x}, {y})"
        
        # 실제 좌표도 표시
        if self.detector:
            real_coords = self.detector.image_to_real_coords(x, y)
            if real_coords:
                real_x, real_y = real_coords
                coord_text += f" → ({real_x:.3f}, {real_y:.3f})m"
        
        self.coord_label.setText(coord_text)
        self.mouse_pos = (x, y)
    
    def convert_label_to_image_coords(self, label_pos: QPoint) -> Optional[Tuple[int, int]]:
        """라벨 좌표를 이미지 좌표로 변환"""
        if not self.detector or self.detector.current_frame is None:
            return None
        
        # 라벨 크기
        label_w = self.image_label.width()
        label_h = self.image_label.height()
        
        # 이미지 크기
        img_h, img_w = self.detector.current_frame.shape[:2]
        
        # 스케일 계산 (KeepAspectRatio)
        scale_w = label_w / img_w
        scale_h = label_h / img_h
        scale = min(scale_w, scale_h)
        
        # 실제 표시 크기
        displayed_w = int(img_w * scale)
        displayed_h = int(img_h * scale)
        
        # 중앙 정렬 오프셋
        offset_x = (label_w - displayed_w) // 2
        offset_y = (label_h - displayed_h) // 2
        
        # 라벨 좌표에서 오프셋 제거
        x_in_display = label_pos.x() - offset_x
        y_in_display = label_pos.y() - offset_y
        
        # 범위 체크
        if x_in_display < 0 or y_in_display < 0 or x_in_display >= displayed_w or y_in_display >= displayed_h:
            return None
        
        # 이미지 좌표로 변환
        img_x = int(x_in_display / scale)
        img_y = int(y_in_display / scale)
        
        return img_x, img_y
    
    def update_image(self, frame):
        """이미지 업데이트"""
        if frame is None:
            return
        
        # 마우스 위치 표시
        display_frame = frame.copy()
        if self.mouse_pos and not self.calibration_mode:
            x, y = self.mouse_pos
            cv2.circle(display_frame, (x, y), 5, (255, 255, 255), 2)
            cv2.circle(display_frame, (x, y), 3, (0, 0, 0), -1)
        
        # 칼리브레이션 점 표시
        if self.calibration_mode:
            for i, (x, y) in enumerate(self.calibration_points):
                cv2.circle(display_frame, (x, y), 8, (0, 255, 255), -1)
                cv2.putText(display_frame, str(i+1), (x+10, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Qt 이미지로 변환
        rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        
        # 라벨에 표시
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.image_label.setPixmap(scaled_pixmap)
    
    def update_ui(self):
        """UI 정기 업데이트"""
        if not self.detector:
            return
        
        # FPS 및 검출 수 업데이트
        self.fps_label.setText(f"FPS: {self.detector.fps:.1f}")
        self.detection_label.setText(f"검출: {self.detector.detection_count}")
        
        # 로봇 상태 업데이트
        self.update_robot_status()
        
        # 가동 시간 업데이트
        if hasattr(self, 'start_time'):
            uptime = time.time() - self.start_time
            hours = int(uptime // 3600)
            minutes = int((uptime % 3600) // 60)
            seconds = int(uptime % 60)
            self.uptime_label.setText(f"가동 시간: {hours:02d}:{minutes:02d}:{seconds:02d}")
        
        # 총 검출 수 업데이트 (예시)
        if hasattr(self, 'total_detections'):
            self.total_detections += self.detector.detection_count
            self.total_detections_label.setText(f"총 검출: {self.total_detections}")
    
    def update_robot_status(self):
        """로봇 상태 리스트 업데이트"""
        self.robot_status_list.clear()
        current_time = time.time()
        
        for robot_id, config in self.robot_configs.items():
            status_text = f"{config.name} (ID:{robot_id})"
            
            if config.active:
                pose = self.detector.get_robot_pose(robot_id) if self.detector else None
                if pose:
                    x, y, yaw = pose
                    status_text += f"\n  위치: ({x:.2f}, {y:.2f})"
                    status_text += f"\n  방향: {yaw*180/pi:.1f}°"
                    
                    # 마지막 검출 시간
                    if config.last_seen > 0:
                        elapsed = current_time - config.last_seen
                        if elapsed < 1.0:
                            status_text += f"\n  상태: 정상 ({elapsed:.1f}s 전)"
                        else:
                            status_text += f"\n  상태: 미검출 ({elapsed:.1f}s 전)"
                    else:
                        status_text += "\n  상태: 미검출"
                else:
                    status_text += "\n  상태: 미검출"
            else:
                status_text += "\n  상태: 비활성"
            
            item = QListWidgetItem(status_text)
            
            # 상태에 따른 색상 설정
            if config.active and config.last_seen > 0 and (current_time - config.last_seen) < 2.0:
                item.setBackground(QColor(200, 255, 200))  # 밝은 초록
            elif config.active:
                item.setBackground(QColor(255, 255, 200))  # 밝은 노랑
            else:
                item.setBackground(QColor(220, 220, 220))  # 회색
            
            self.robot_status_list.addItem(item)
    
    def log_message(self, message: str):
        """로그 메시지 추가"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.log_text.append(log_entry)
        
        # 자동 스크롤
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def clear_log(self):
        """로그 지우기"""
        self.log_text.clear()
        self.log_message("로그 지워짐")
    
    def save_config(self):
        """설정 저장"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "설정 저장", "robot_monitor_config.json", "JSON Files (*.json)"
        )
        
        if filename:
            try:
                config_data = {
                    'camera_config': asdict(self.camera_config),
                    'robot_configs': {
                        str(k): asdict(v) for k, v in self.robot_configs.items()
                    }
                }
                
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(config_data, f, indent=2, ensure_ascii=False)
                
                self.log_message(f"설정 저장됨: {filename}")
                QMessageBox.information(self, "성공", "설정이 저장되었습니다.")
                
            except Exception as e:
                QMessageBox.critical(self, "오류", f"설정 저장 실패:\n{e}")
    
    def load_config(self):
        """설정 로드"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "설정 로드", "", "JSON Files (*.json)"
        )
        
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                
                # 카메라 설정 로드
                if 'camera_config' in config_data:
                    camera_data = config_data['camera_config']
                    self.camera_config = CameraConfig(**camera_data)
                
                # 로봇 설정 로드
                if 'robot_configs' in config_data:
                    robot_data = config_data['robot_configs']
                    self.robot_configs = {}
                    for k, v in robot_data.items():
                        robot_id = int(k)
                        # tuple로 저장된 color를 복원
                        if 'color' in v and isinstance(v['color'], list):
                            v['color'] = tuple(v['color'])
                        self.robot_configs[robot_id] = RobotConfig(**v)
                
                # UI 업데이트
                self.robot_tabs.clear()
                for robot_id, config in self.robot_configs.items():
                    widget = RobotConfigWidget(config)
                    self.robot_tabs.addTab(widget, f"로봇-{robot_id}")
                
                self.log_message(f"설정 로드됨: {filename}")
                QMessageBox.information(self, "성공", "설정이 로드되었습니다.")
                
            except Exception as e:
                QMessageBox.critical(self, "오류", f"설정 로드 실패:\n{e}")
    
    def closeEvent(self, event):
        """윈도우 종료 이벤트"""
        try:
            self.stop_monitoring()
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        event.accept()


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    
    # 다크 테마 설정 (선택사항)
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(0, 0, 0))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)
    
    # 메인 윈도우 생성 및 실행
    window = MonitorMainWindow()
    window.show()
    
    # 종료 처리
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("🛑 사용자 종료 요청")
    except Exception as e:
        print(f"❌ 예상치 못한 오류: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ 프로그램 종료")


if __name__ == '__main__':
    main()