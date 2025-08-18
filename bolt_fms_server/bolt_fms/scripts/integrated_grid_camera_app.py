#!/usr/bin/env python3
"""
통합 그리드 카메라 및 작업 관리 앱
- 탭 1: 카메라 영상 위에 그리드 표시 및 로봇 제어
- 탭 2: 작업 스케줄 관리 시스템
"""
import os
import sys
import time
import math
import heapq
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Set, Any
from enum import Enum
from queue import PriorityQueue

import numpy as np
import requests

# === OpenCV / AprilTags ===
try:
    import cv2
except Exception:  # OpenCV 미설치 환경에서도 코드 로드는 되게
    cv2 = None
from pupil_apriltags import Detector

# === PySide6 ===
from PySide6.QtCore import (
    Qt, QObject, QThread, QTimer, Signal, Slot, QPoint, QPointF, QRectF, QSize
)
from PySide6.QtGui import (
    QImage, QPixmap, QPainter, QPen, QAction, QBrush, QColor, QFont, QPolygonF
)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QLabel, QPushButton, 
    QVBoxLayout, QHBoxLayout, QGridLayout, QSpinBox, QListWidget, QListWidgetItem, 
    QCheckBox, QGroupBox, QComboBox, QTabWidget, QTableWidget, QTableWidgetItem, 
    QStatusBar, QTextEdit, QLineEdit, QGraphicsScene, QGraphicsView, 
    QGraphicsRectItem, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsPolygonItem
)

# === ROS 2 ===
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import String, Float32
from builtin_interfaces.msg import Time


# 로봇 ID와 AprilTag ID 매핑
ROBOT_CONFIG = {
    1: 4,
    2: 2,
    3: 563,
    4: 0,
    5: 0,
}

RACK_CONFIG = [
    {
        "rack_id": 1,
        "rows": 2,
        "cols": 2,
        "floors": 3,
        "cell_width_m": 0.0833,
        "cell_height_m": 0.1666,
        "margin_m": 0.166,
        "x_m": 1.2,
        "y_m": 0.33,
        "initial_occupied": [
        ]
    },
    {
        "rack_id": 2,
        "rows": 2,
        "cols": 2,
        "floors": 3,
        "cell_width_m": 0.0833,
        "cell_height_m": 0.1666,
        "margin_m": 0.166,
        "x_m": 1.53,
        "y_m": 0.33,
        "initial_occupied": [
            (0,1,0),(1,1,0),
            (0,1,1),(1,1,1),
        ]
    }
]

class MyLocation:
    """
    로봇의 목표 포즈 정보를 담는 클래스입니다.
    """
    def __init__(self, rack_id: int, floor: int, row: int, col: int, pose_m: Tuple[float, float, float], direction: str = None):
        """
        Location 객체의 생성자입니다.
        
        Args:
            rack_id (int): 랙의 고유 ID
            floor (int): 랙의 층
            row (int): 랙의 행
            col (int): 랙의 열
            pose_m (Tuple[float, float, float]): 로봇의 물리적 위치 (x, y, yaw) (단위: 미터, 라디안)
            direction (str): 로봇이 바라보는 방향 ('up', 'down', 'left', 'right')
        """
        self.rack_id = rack_id
        self.floor = floor
        self.row = row
        self.col = col
        self.pose_m = pose_m
        self.direction = direction

    def __repr__(self) -> str:
        """
        객체를 문자열로 표현하여 출력할 때 사용합니다.
        """
        return (f"Location(rack_id={self.rack_id}, floor={self.floor}, row={self.row}, col={self.col}, "
                f"pose_m=({self.pose_m[0]:.2f}m, {self.pose_m[1]:.2f}m, {self.pose_m[2]:.2f}rad), direction='{self.direction}')")

class MyRack:
    def __init__(self, rack_id: int, rows: int, cols: int, floors: int, cell_width_m: float, cell_height_m: float, margin_m: float, x_m: float, y_m: float):
        self.rack_id = rack_id
        self.rows = rows
        self.cols = cols
        self.floors = floors
        self.cell_width_m = cell_width_m
        self.cell_height_m = cell_height_m
        self.margin_m = margin_m
        self.x_m = x_m
        self.y_m = y_m
        self.locations: List[List[List[bool]]] = [
            [[False for _ in range(cols)] for _ in range(rows)] for _ in range(floors)
        ]
        self.reservation: List[List[List[bool]]] = [
            [[False for _ in range(cols)] for _ in range(rows)] for _ in range(floors)
        ]

class RackManager:
    def __init__(self):
        self.racks: Dict[int, MyRack] = {}

    def add_rack(self, rack_obj: MyRack):
        self.racks[rack_obj.rack_id] = rack_obj

    def get_rack(self, rack_id: int) -> MyRack | None:
        return self.racks.get(rack_id)

    def get_all_racks(self) -> Dict[int, MyRack]:
        return self.racks
    
    def is_location_available(self, rack_id: int, floor: int, row: int, col: int) -> bool:
        rack = self.racks.get(rack_id)
        if rack and floor < rack.floors and row < rack.rows and col < rack.cols:
            return not rack.locations[floor][row][col]
        return False
    
    def is_reservation_available(self, rack_id: int, floor: int, row: int, col: int) -> bool:
        rack = self.racks.get(rack_id)
        if rack and floor < rack.floors and row < rack.rows and col < rack.cols:
            return not rack.reservation[floor][row][col]
        return False

    def find_first_available_space(self) -> Optional[Tuple[int, int, int, int]]:
        """모든 랙을 순회하며 첫 번째 가용 공간을 찾습니다.
        반환 값: (rack_id, floor, row, col) 형태의 튜플
        """
        """모든 랙을 순회하며 첫 번째 가용 공간을 찾습니다.
        반환 값: (rack_id, floor, row, col) 형태의 튜플
        """
        sorted_racks = sorted(self.get_all_racks().values(), key=lambda r: r.rack_id)
        
        for rack in sorted_racks:
            for floor in range(rack.floors):
                for row in range(rack.rows):
                    for col in range(rack.cols):
                        if self.is_reservation_available(rack.rack_id, floor, row, col):
                            return (rack.rack_id, floor, row, col)
        
        return None
    
import random
import math
class StockVisualizerWidget(QWidget):
    def __init__(self, rack_manager: RackManager):
        super().__init__()
        self.rack_manager = rack_manager
        self.setWindowTitle("랙 재고 상태 시각화 (층별) - 좌측 하단 원점")
        self.setGeometry(100, 100, 800, 600)
        
        self._candidate_index = 0  # 순환 커서

        self.SCALE_FACTOR = 500 # 1미터 = 100픽셀
        self.SCENE_HEIGHT_PX = 500 # Y좌표 뒤집기를 위한 가상 씬 높이
        self.ROBOT_CLEARANCE_M = 0.083
        self.ROBOT_CLEARANCE_PX = self.ROBOT_CLEARANCE_M * self.SCALE_FACTOR
        self.ROBOT_RADIUS_PX = 10
        
        self.current_floor = 0
        self.highlight_pose: Optional[Tuple[int, int, int, int]] = None
        self.robot_physical_pose_px: Optional[QPointF] = None
        self.robot_physical_pose_m: Optional[QPointF] = None
        self.robot_direction: Optional[str] = None
        
        main_layout = QVBoxLayout(self)
        
        control_layout = QHBoxLayout()
        self.floor_selector = QComboBox(self)
        self.floor_selector.currentIndexChanged.connect(self.update_visualization)
        self.floor_selector.setMinimumWidth(100)
        
        self.find_space_button = QPushButton("가용 공간 찾기")
        self.find_space_button.clicked.connect(self.find_and_display_pose)
        
        # self.find_space_button1 = QPushButton("로봇 위치 찾기")
        # self.find_space_button1.clicked.connect(self.get_inbound_target_pose)

        self.pose_label = QLabel("로봇 위치 (Pose): -")
        self.pose_label.setMinimumWidth(300)

        control_layout.addWidget(QLabel("층 선택:"))
        control_layout.addWidget(self.floor_selector)
        control_layout.addWidget(self.find_space_button)
        # control_layout.addWidget(self.find_space_button1)
        control_layout.addStretch(1)
        control_layout.addWidget(self.pose_label)
        
        main_layout.addLayout(control_layout)
        
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene, self)
        main_layout.addWidget(self.view)
        
        self.populate_floor_selector()
        self.update_visualization()

    def populate_floor_selector(self):
        max_floors = 0
        for rack in self.rack_manager.get_all_racks().values():
            if rack.floors > max_floors:
                max_floors = rack.floors
                
        for f in range(max_floors):
            self.floor_selector.addItem(f"지상 {f + 1}층")
    
    def find_and_display_pose(self):
        self.highlight_pose = self.rack_manager.find_first_available_space()
        
        if self.highlight_pose:
            rack_id, floor, row, col = self.highlight_pose
            
            found_rack_obj = self.rack_manager.get_rack(rack_id)
            if not found_rack_obj:
                self.pose_label.setText("로봇 위치 (Pose): 랙을 찾을 수 없습니다.")
                return

            self.robot_physical_pose_px, self.robot_direction = self.find_safe_border_pose(
                rack_id, row, col
            )
            
            z_pos = floor + 1
            
            if self.robot_physical_pose_px:
                self.robot_physical_pose_m = QPointF(
                    self.robot_physical_pose_px.x() / self.SCALE_FACTOR,
                    self.robot_physical_pose_px.y() / self.SCALE_FACTOR
                )
                self.pose_label.setText(
                    f"로봇 위치(Pose): 랙 {rack_id} / 층 {z_pos} / ({row}, {col})\n"
                    f"바라보는 방향: {self.robot_direction}\n"
                    f"물리적 위치 (좌하단 원점): (X={self.robot_physical_pose_m.x():.2f}m, Y={self.robot_physical_pose_m.y():.2f}m)\n"
                    f" (픽셀: X={self.robot_physical_pose_px.x():.1f}, Y={self.robot_physical_pose_px.y():.1f})"
                )
            else:
                 self.pose_label.setText("로봇 위치 (Pose): 안전한 위치를 찾을 수 없습니다.")

            found_rack_obj.locations[floor][row][col] = True
        else:
            self.pose_label.setText("로봇 위치 (Pose): 가용 가능한 공간이 없습니다.")
            self.robot_physical_pose_px = None
            self.robot_physical_pose_m = None
            self.robot_direction = None
            
        self.update_visualization()
    
    def find_safe_border_pose(self, target_rack_id: int, row: int, col: int) -> Tuple[Optional[QPointF], Optional[str]]:
        """
        주어진 칸의 경계선 바깥에 위치하며 다른 랙과 겹치지 않는 안전한 로봇 위치를 찾습니다.
        좌표계는 좌측 하단을 원점으로 합니다.
        우선순위: 오른쪽 > 왼쪽 > 아래 > 위
        """
        found_rack_obj = self.rack_manager.get_rack(target_rack_id)
        
        # 충돌 검사를 위해 Qt의 좌상단 원점 좌표계로 변환된 랙 경계 정보를 생성
        rack_boundaries_qt = []
        for rack in self.rack_manager.get_all_racks().values():
            rack_width_px = rack.cols * (rack.cell_width_m * self.SCALE_FACTOR)
            rack_height_px = rack.rows * (rack.cell_height_m * self.SCALE_FACTOR)
            rack_x_px = rack.x_m * self.SCALE_FACTOR
            rack_y_px = rack.y_m * self.SCALE_FACTOR
            
            # 좌하단 기준 Y -> 좌상단 기준 Y로 변환하여 QRectF 생성
            qt_rack_y = self.SCENE_HEIGHT_PX - (rack_y_px + rack_height_px)
            
            rack_rect_qt = QRectF(rack_x_px, qt_rack_y, rack_width_px, rack_height_px)
            rack_boundaries_qt.append((rack.rack_id, rack_rect_qt))
        
        cell_width_px = found_rack_obj.cell_width_m * self.SCALE_FACTOR
        cell_height_px = found_rack_obj.cell_height_m * self.SCALE_FACTOR
        
        # 랙의 절대 위치(픽셀, 좌하단 원점)
        rack_x_px = found_rack_obj.x_m * self.SCALE_FACTOR
        rack_y_px = found_rack_obj.y_m * self.SCALE_FACTOR
        
        candidate_poses = []
        
        # 오른쪽
        x_pos = rack_x_px + (col + 1) * cell_width_px + self.ROBOT_CLEARANCE_PX
        y_pos = rack_y_px + row * cell_height_px + cell_height_px / 2
        candidate_poses.append((QPointF(x_pos, y_pos), 'right'))
        
        # 왼쪽
        x_pos = rack_x_px + col * cell_width_px - self.ROBOT_CLEARANCE_PX
        y_pos = rack_y_px + row * cell_height_px + cell_height_px / 2
        candidate_poses.append((QPointF(x_pos, y_pos), 'left'))
        
        # 아래쪽
        x_pos = rack_x_px + col * cell_width_px + cell_width_px / 2
        y_pos = rack_y_px + row * cell_height_px - self.ROBOT_CLEARANCE_PX
        candidate_poses.append((QPointF(x_pos, y_pos), 'down'))

        # 위쪽
        x_pos = rack_x_px + col * cell_width_px + cell_width_px / 2
        y_pos = rack_y_px + (row + 1) * cell_height_px + self.ROBOT_CLEARANCE_PX
        candidate_poses.append((QPointF(x_pos, y_pos), 'up'))
        
        for pose, direction in candidate_poses:
            # 충돌 검사를 위해 후보 위치(좌하단 원점)를 Qt의 좌상단 원점 기준으로 변환
            qt_pose_y = self.SCENE_HEIGHT_PX - pose.y()
            robot_rect_qt = QRectF(pose.x() - self.ROBOT_RADIUS_PX, qt_pose_y - self.ROBOT_RADIUS_PX, self.ROBOT_RADIUS_PX * 2, self.ROBOT_RADIUS_PX * 2)
            
            is_safe = True
            for _, rack_rect_qt in rack_boundaries_qt:
                if robot_rect_qt.intersects(rack_rect_qt):
                    is_safe = False
                    break
            if is_safe:
                return pose, direction # 안전한 위치는 좌하단 원점 기준으로 반환
        
        # 모든 경계가 다른 랙과 겹치는 경우, 통로 중앙으로 폴백합니다.
        aisle_width_px_fallback = found_rack_obj.margin_m * self.SCALE_FACTOR
        x_pos = rack_x_px + found_rack_obj.cols * cell_width_px + aisle_width_px_fallback / 2
        y_pos = rack_y_px + row * cell_height_px + cell_height_px / 2
        return QPointF(x_pos, y_pos), 'left'

    def update_visualization(self):
        self.scene.clear()
        self.current_floor = self.floor_selector.currentIndex()
        
        font = QFont("Arial", 10)
        
        for rack in self.rack_manager.get_all_racks().values():
            if self.current_floor >= rack.floors:
                continue
            
            # 랙의 절대 위치(픽셀, 좌하단 원점)
            rack_x_px = rack.x_m * self.SCALE_FACTOR
            rack_y_px = rack.y_m * self.SCALE_FACTOR
            
            cell_width_px = rack.cell_width_m * self.SCALE_FACTOR
            cell_height_px = rack.cell_height_m * self.SCALE_FACTOR
            
            for r in range(rack.rows):
                for c in range(rack.cols):
                    grid_x = rack_x_px + c * cell_width_px
                    
                    # Y좌표를 좌하단 원점 기준으로 계산 후, 그리기를 위해 좌상단 원점 기준으로 변환
                    cell_y_bl = rack_y_px + r * cell_height_px # 셀의 좌하단 Y
                    grid_y = self.SCENE_HEIGHT_PX - (cell_y_bl + cell_height_px) # 그리기 위한 좌상단 Y
                    
                    is_occupied = rack.locations[self.current_floor][r][c]
                    
                    rect = QGraphicsRectItem(grid_x, grid_y, cell_width_px, cell_height_px)
                    
                    if self.highlight_pose and self.highlight_pose == (rack.rack_id, self.current_floor, r, c):
                        rect.setBrush(QBrush(QColor(0, 150, 0, 100)))
                        rect.setPen(QPen(QColor(255, 0, 0), 3))
                    elif is_occupied:
                        rect.setBrush(QBrush(QColor(0, 150, 0)))
                        rect.setPen(QPen(Qt.black))
                    else:
                        rect.setBrush(QBrush(QColor(200, 255, 200)))
                        rect.setPen(QPen(Qt.black))

                    self.scene.addItem(rect)
                    
                    text = f"랙 {rack.rack_id}\n({r}, {c})"
                    text_item = QGraphicsTextItem(text)
                    text_item.setPos(grid_x + cell_width_px / 8, grid_y + cell_height_px / 4)
                    text_item.setFont(font)
                    text_item.setDefaultTextColor(QColor(0, 0, 0))
                    self.scene.addItem(text_item)

        # --- 로봇 시각화 부분 시작 ---
        if self.robot_physical_pose_px:
            # 로봇의 위치(좌하단 원점)를 그리기를 위해 좌상단 원점 기준으로 변환
            robot_center_x = self.robot_physical_pose_px.x()
            robot_center_y_qt = self.SCENE_HEIGHT_PX - self.robot_physical_pose_px.y()
            
            robot = QGraphicsEllipseItem(
                robot_center_x - self.ROBOT_RADIUS_PX,
                robot_center_y_qt - self.ROBOT_RADIUS_PX,
                self.ROBOT_RADIUS_PX * 2,
                self.ROBOT_RADIUS_PX * 2
            )
            robot.setBrush(QBrush(QColor(0, 100, 255)))
            robot.setPen(QPen(Qt.black, 1))
            self.scene.addItem(robot)
            
            arrow_size = 10
            arrow = QGraphicsPolygonItem()
            polygon = QPolygonF()
            
            if self.robot_direction == 'left':
                polygon.append(QPointF(robot_center_x - self.ROBOT_RADIUS_PX, robot_center_y_qt))
                polygon.append(QPointF(robot_center_x - self.ROBOT_RADIUS_PX - arrow_size, robot_center_y_qt - arrow_size / 2))
                polygon.append(QPointF(robot_center_x - self.ROBOT_RADIUS_PX - arrow_size, robot_center_y_qt + arrow_size / 2))
            elif self.robot_direction == 'right':
                polygon.append(QPointF(robot_center_x + self.ROBOT_RADIUS_PX, robot_center_y_qt))
                polygon.append(QPointF(robot_center_x + self.ROBOT_RADIUS_PX + arrow_size, robot_center_y_qt - arrow_size / 2))
                polygon.append(QPointF(robot_center_x + self.ROBOT_RADIUS_PX + arrow_size, robot_center_y_qt + arrow_size / 2))
            elif self.robot_direction == 'up':
                polygon.append(QPointF(robot_center_x, robot_center_y_qt - self.ROBOT_RADIUS_PX))
                polygon.append(QPointF(robot_center_x - arrow_size / 2, robot_center_y_qt - self.ROBOT_RADIUS_PX - arrow_size))
                polygon.append(QPointF(robot_center_x + arrow_size / 2, robot_center_y_qt - self.ROBOT_RADIUS_PX - arrow_size))
            elif self.robot_direction == 'down':
                polygon.append(QPointF(robot_center_x, robot_center_y_qt + self.ROBOT_RADIUS_PX))
                polygon.append(QPointF(robot_center_x - arrow_size / 2, robot_center_y_qt + self.ROBOT_RADIUS_PX + arrow_size))
                polygon.append(QPointF(robot_center_x + arrow_size / 2, robot_center_y_qt + self.ROBOT_RADIUS_PX + arrow_size))
            
            if not polygon.isEmpty():
                arrow.setPolygon(polygon)
                arrow.setBrush(QBrush(QColor(255, 0, 0)))
                arrow.setPen(QPen(Qt.red, 1))
                self.scene.addItem(arrow)
        # --- 로봇 시각화 부분 끝 ---
            
        self.view.setSceneRect(self.scene.itemsBoundingRect())
   
    def set_location_occupied(self, rack_id: int, floor: int, row: int, col: int, occupied: bool):
        """
        주어진 랙의 특정 위치의 점유 상태를 변경합니다.
        """
        rack = self.rack_manager.get_rack(rack_id)
        if rack and floor < rack.floors and row < rack.rows and col < rack.cols:
            rack.locations[floor][row][col] = occupied
            self.update_visualization()
            
    def set_reservation_occupied(self, rack_id: int, floor: int, row: int, col: int, occupied: bool):
        """
        주어진 랙의 특정 위치의 점유 상태를 변경합니다.
        """
        rack = self.rack_manager.get_rack(rack_id)
        if rack and floor < rack.floors and row < rack.rows and col < rack.cols:
            rack.reservation[floor][row][col] = occupied
            # self.update_visualization()

    def get_inbound_target_pose(self) -> MyLocation:
        """
        가장 먼저 발견되는 가용 공간을 찾아 로봇의 목표 포즈를 계산하여 반환합니다.
        반환 값:
        - 성공 시: Location
        - 실패 시: None
        """
        highlight_pose = self.rack_manager.find_first_available_space()
        
        if not highlight_pose:
            print("가용 가능한 공간이 없습니다.")
            return None

        rack_id, floor, row, col = highlight_pose
        
        robot_physical_pose_px, robot_direction = self.find_safe_border_pose(
            rack_id, row, col
        )

        if not robot_physical_pose_px:
            print("안전한 로봇 위치를 찾을 수 없습니다.")
            return None

        # 픽셀을 미터로 변환 (좌하단 원점 기준)
        x_m = round(robot_physical_pose_px.x() / self.SCALE_FACTOR, 2)
        y_m = round(robot_physical_pose_px.y() / self.SCALE_FACTOR, 2)
        
        # 방향을 yaw 각도로 변환
        yaw_rad = 0.0
        if robot_direction == 'up':
            yaw_rad = math.radians(90)
        elif robot_direction == 'down':
            yaw_rad = math.radians(-90)
        elif robot_direction == 'left':
            yaw_rad = math.radians(180)
        elif robot_direction == 'right':
            yaw_rad = math.radians(0)
        yaw_rad = round(yaw_rad, 2)

        self.set_reservation_occupied(rack_id, floor, row, col, True)
        location = MyLocation(rack_id=rack_id, floor=floor, row=row, col=col, pose_m=(x_m,y_m,yaw_rad),direction=robot_direction)
        return location

    def get_outbound_target_pose(self) -> MyLocation:
        """
        순환 방식으로 후보 좌표를 하나 선택하고,
        해당 랙 정보 기반으로 로봇의 물리적 위치를 계산하여 MyLocation 반환
        """
            # 후보 좌표 순환용
        candidate_positions = [
            (0, 1, 0),
            (1, 1, 0),
            (0, 1, 1),
            (1, 1, 1),
        ]

        # 1️⃣ 후보 좌표 순환
        row, col, floor = candidate_positions[self._candidate_index]
        self._candidate_index = (self._candidate_index + 1) % len(candidate_positions)

        # 2️⃣ 실제 존재하는 랙 중 하나 (예: rack_id=2)
        rack: MyRack = list(self.rack_manager.racks.values())[1]  # 두 번째 랙 (rack_id=2)
        rack_id = rack.rack_id

        # 3️⃣ 안전한 로봇 위치 계산
        robot_physical_pose_px, robot_direction = self.find_safe_border_pose(
            rack_id, row, col
        )
        if not robot_physical_pose_px:
            print("❌ 안전한 로봇 위치를 찾을 수 없습니다.")
            return None

        # 픽셀 → 미터 변환
        x_m = round(robot_physical_pose_px.x() / self.SCALE_FACTOR, 2)
        y_m = round(robot_physical_pose_px.y() / self.SCALE_FACTOR, 2)

        # 방향 → yaw 변환
        yaw_rad = 0.0
        if robot_direction == 'up':
            yaw_rad = math.radians(90)
        elif robot_direction == 'down':
            yaw_rad = math.radians(-90)
        elif robot_direction == 'left':
            yaw_rad = math.radians(180)
        elif robot_direction == 'right':
            yaw_rad = math.radians(0)
        yaw_rad = round(yaw_rad, 2)

        # 예약 처리
        rack.reservation[floor][row][col] = True

        # 4️⃣ 최종 MyLocation 반환
        return MyLocation(
            rack_id=rack_id,
            floor=floor,
            row=row,
            col=col,
            pose_m=(x_m, y_m, yaw_rad),
            direction=robot_direction
        )


# Task 클래스에 가벼운 메타 필드 추가 (기존 코드와 역호환)
class Task:
    def __init__(
        self,
        task_id,
        task_type: str,
        robot_type: str,
        location : MyLocation = None,
        priority=1,
        meta: dict | None = None,
    ):
        self.task_id = task_id
        self.task_type = task_type
        self.robot_type = robot_type
        self.location = location
        self.priority = priority
        self.status = "PENDING"
        self.assigned_robot = None
        self.meta = meta or {}

    def __lt__(self, other):
        return self.priority < other.priority
    
    def complete_task(self):
        """작업 완료 처리"""
        self.status = "COMPLETED"

class Robot:
    """로봇의 상태와 정보를 담는 클래스"""

    def __init__(
        self,
        robot_id: int,
        tag_id: int,
        robot_type: str,
        current_pose: tuple = (0.0, 0.0, 0.0),
        current_task : Task = None,
        battery: float = 100.0,
        status: str = "PENDING",
        joint_angles: list = None,
    ):
        self.robot_id = robot_id
        self.tag_id = tag_id
        self.robot_type = robot_type
        self.current_pose = current_pose  # (x, y, yaw)
        self.current_task = Task(-1,task_type="IDLE",robot_type=robot_type,location=None)  # (x, y, yaw)
        self.battery = battery
        self.status = status  # "IDLE", "BUSY", "CHARGING" 등
        self.joint_angles = joint_angles or []

    def __repr__(self):
        return f"Robot(ID: {self.robot_id}, Tag: {self.tag_id}, Type: {self.robot_type}, Status: {self.status})"

    def is_available(self) -> bool:
        """로봇이 작업 가능한 상태인지 확인"""
        return self.current_task.task_type == "IDLE" and self.status == "PENDING"

    def update_pose(self, x: float, y: float, yaw: float):
        """로봇의 현재 포즈 업데이트"""
        self.current_pose = (x, y, yaw)

    def assign_task(self, task):
        """로봇에게 새로운 작업 할당"""
        self.current_task = task

    def complete_task(self):
        """현재 작업 완료 처리"""
        self.current_task.status = "COMPLETED"
        self.current_task = Task(-1,task_type="IDLE",robot_type=self.robot_type,location=None)  # (x, y, yaw)

    def create_idle_task(self):
        self.current_task = Task(-1,task_type="IDLE",robot_type=self.robot_type,location=None)  # (x, y, yaw)


# ======================================================================
# ROS2 통신 관련 클래스들
# ======================================================================
from bolt_fms.msg import TaskSimple  # (A) 선택 시


class RobotStatusNode(Node):
    """로봇 위치 정보를 ROS2 토픽으로 발행하는 노드"""

    def __init__(self, robots : dict = None):
        super().__init__("robot_publisher")

        self.robots = robots if robots is not None else {}
        
        for robot_id in robots.keys():
            # 상태 구독자 (현재 상태)
            status_topic = f"/robot{robot_id}/task_status"
            self.create_subscription(
                String,
                status_topic,
                lambda msg, robot_id=robot_id: self.status_callback(msg, robot_id),
                10
            )
            self.get_logger().info(f"Created pose subscriber for {status_topic}")

            # 카메라 포즈 구독자 (현재 위치)
            battery_topic = f"/robot{robot_id}/battery"
            self.create_subscription(
                Float32,
                battery_topic,
                lambda msg, robot_id=robot_id: self.battery_callback(msg, robot_id),
                10
            )
            self.get_logger().info(f"Created pose subscriber for {battery_topic}")
            
            if robot_id in self.robots:
                if self.robots[robot_id].robot_type == "MOBILE":
                    self.robots[robot_id].status = "PENDING"
                    self.robots[robot_id].battery = 100.0

        # 각 로봇에 대한 퍼블리셔 생성
        self.pose_publishers = {}
        self.target_pose_publishers = {}
        self.cmd_vel_publishers = {}
        self.task_publishers = {}

        for robot_id in ROBOT_CONFIG.keys():
            # 카메라 포즈 퍼블리셔 (현재 위치)
            pose_topic = f"/robot{robot_id}/camera_pose"
            pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
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

            task_topic = f"/robot{robot_id}/task"
            task_pub = self.create_publisher(TaskSimple, task_topic, 10)  # robot_id별로
            self.task_publishers[robot_id] = task_pub
            self.get_logger().info(f"Create task simple publisher for {task_topic}")

    # 로봇 구독 ==============================================================================
    def status_callback(self, msg, robot_id):
        """로봇의 상태 메시지를 수신하면 호출되는 콜백 함수"""
        if robot_id in self.robots:
            # ❌ Robot 객체의 battery 속성에 실제 값을 할당
            self.robots[robot_id].status = msg.data

    def battery_callback(self, msg, robot_id):
        """로봇의 배터리 메시지를 수신하면 호출되는 콜백 함수"""
        if robot_id in self.robots:
            # ❌ Robot 객체의 battery 속성에 실제 값을 할당
            self.robots[robot_id].battery = msg.data
            # self.get_logger().info(f"Robot {robot_id} battery updated: {msg.data:.2f}%")
        
    # 로봇 발행 ==============================================================================
    def publish_robot_pose(self, robot_id, x, y, yaw):
        """로봇 위치를 PoseStamped 메시지로 발행"""
        if robot_id not in self.pose_publishers:
            return

        # PoseStamped 메시지 생성
        pose_msg = PoseStamped()

        # Header 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Position 설정
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.z = float(math.sin(yaw / 2.0))
        pose_msg.pose.orientation.w = float(math.cos(yaw / 2.0))

        # 메시지 발행
        # if(robot_id == 2):
        #     self.get_logger().error(f"camera_pose x={x}, y={y}, yaw={yaw}")
        publisher = self.pose_publishers.get(robot_id)
        if publisher:
            publisher.publish(pose_msg)
        # print(self.pose_publishers[robot_id])

    def publish_task(
        self, robot_id: int, task_type: str, x: float, y: float, yaw: float, pinky_id: int = None
    ):
        msg = TaskSimple()
        msg.task_type = task_type
        msg.target_pose.header.frame_id = f"pinky{robot_id}/base_link"
        now = self.get_clock().now().to_msg()
        msg.target_pose.header.stamp = now
        msg.target_pose.pose.position.x = x
        msg.target_pose.pose.position.y = y
        msg.target_pose.pose.position.z = 0.0

        msg.target_pose.pose.orientation.z = float(math.sin(yaw / 2.0))
        msg.target_pose.pose.orientation.w = float(math.cos(yaw / 2.0))

        msg.pinky_id = pinky_id

        pub = self.task_publishers.get(robot_id)
        if pub:
            print("task publish")
            pub.publish(msg)

    def publish_target_pose(self, robot_id, x, y, yaw):
        """목표 위치(웨이포인트)를 ROS2 토픽으로 발행"""
        if robot_id not in self.target_pose_publishers:
            return

        # PoseStamped 메시지 생성
        pose_msg = PoseStamped()

        # Header 설정
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Position 설정
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)

        # 메시지 발행
        self.get_logger().error(f"target_pose x={x}, y={y}, yaw={yaw}")
        self.target_pose_publishers[robot_id].publish(pose_msg)

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


# ======================================================================
# 작업 관리 시스템 클래스들
# ======================================================================


class TaskStatus(Enum):
    PENDING = "Pending"
    ASSIGNED = "Assigned"
    IN_PROGRESS = "In Progress"
    COMPLETED = "Completed"
    FAILED = "Failed"
    CANCELLED = "Cancelled"


class TaskType(Enum):
    MOVE = "move"
    WAIT_USER = "wait_user"
    LOAD = "load : pick & place on boltbot"
    UNLOAD = "unload : pick & place from boltbot"


class RobotType(Enum):
    MOBILE = "mobile"
    ARM = "robot_arm"


class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"
    CHARGING = "Charge"
    LOWBATTERY = "Lowbattery"

class ProcessTask:
    def __init__(self, task_id, steps: list[Task]):
        self.task_id = task_id
        self.steps = steps
        self.current_index = 0
        self.assigned_mobile_robot_id = None

    def current_step(self):
        if self.current_index < len(self.steps):
            return self.steps[self.current_index]
        return None

    def advance(self):
        self.current_index += 1

    def is_done(self):
        return self.current_index >= len(self.steps)

    def is_mobile_robot_locked(self, robot_id):
        return self.assigned_mobile_robot_id == robot_id and not self.is_done()

import select_location

class TaskManager:
    def __init__(self, camera = None):
        self.task_queue = PriorityQueue()
        self.all_process_tasks: list[ProcessTask] = []
        self.camera : GridCameraWidget = camera
        self.robots = self.camera.robots
        self.db = None
        # 초기화 시 한 번
        self.seen_inbound_ids = set()
        self.seen_outbound_ids = set()
        self.on_task_assigned = (
            None
        )         
    
    def add_process_task(self, process: ProcessTask):
        self.all_process_tasks.append(process)
        step = process.current_step()
        if step:
            self.task_queue.put((step.priority, time.time(), step))
    
    # TaskManager 클래스의 assign_tasks_waypoint 메서드
    def assign_tasks_waypoint(self):
        """Taskmanager 작업 할당 실시!"""
        messages = []
        new_queue = PriorityQueue()

        while not self.task_queue.empty(): # 작업큐 가 비어있지 않다면
            _, _, task = self.task_queue.get() # 작업큐 가져오기
            task : Task = task  # 해야할 작업
            # print(type(task))

            process = next((c for c in self.all_process_tasks if task in c.steps), None) # 현재 작업이 속한 프로세스 확인

            if (
                process
                and task.robot_type == "MOBILE"
                and process.assigned_mobile_robot_id
            ):
                robot : Robot = self.robots.get(process.assigned_mobile_robot_id)
                if robot and robot.is_available():
                    assigned = True
                else:
                    robot = None
            else:
                robot = self.select_robot(task)

            if robot:
                task.status = "ASSIGNED"
                task.assigned_robot = robot.robot_id
                # robot.status = task.status
                # robot.current_task.task_type = task.task_type
                robot.assign_task(task) # 로봇에게 작업 할당

                if (
                    task.robot_type == "MOBILE"
                    and process
                    and not process.assigned_mobile_robot_id
                ):
                    process.assigned_mobile_robot_id = robot.robot_id

                messages.append(
                    f"✅ Task {task.task_id} [{task.task_type}] assigned to Robot {robot.robot_id}"
                )

                # 할당된 로봇의 타입과 작업 유형에 따라 경로를 계획합니다.
                if task.robot_type == "MOBILE" and task.task_type == "MOVE_TO_INBOUND":
                    if task.location:
                        loc : MyLocation = task.location
                        
                        goal_pos = (loc.pose_m[0], loc.pose_m[1])  # task 객체에 goal_pos가 있다고 가정
                        final_yaw = loc.pose_m[2] # task 객체에 final_yaw가 있다고 가정

                        # robot.assign_task(task) # 로봇에게 작업 할당

                        self.camera.set_robot_goal(
                            pos=None,
                            robot_id=robot.robot_id,
                            real_pos=goal_pos,
                            final_yaw=final_yaw
                        )
                        self.camera.plan_robot_paths()

                        if robot.robot_id in self.camera.robot_paths:
                            print(f"✔️ 로봇 {robot.robot_id}에게 경로가 계획되었습니다.")
                            print(f"    - 최종 목표: {goal_pos} 위치, {final_yaw} yaw")
                        else:
                            print(f"❌ 로봇 {robot.robot_id}의 경로를 찾을 수 없습니다.")
                elif task.robot_type == "MOBILE" and task.task_type == "MOVE_TO_OUTBOUND":
                    if task.location:
                        loc : MyLocation = task.location
                        
                        goal_pos = (loc.pose_m[0], loc.pose_m[1])  # task 객체에 goal_pos가 있다고 가정
                        final_yaw = loc.pose_m[2] # task 객체에 final_yaw가 있다고 가정
                        
                        # robot.assign_task(task) # 로봇에게 작업 할당

                        self.camera.set_robot_goal(
                            pos=None,
                            robot_id=robot.robot_id,
                            real_pos=goal_pos,
                            final_yaw=final_yaw
                        )
                        self.camera.plan_robot_paths()

                        if robot.robot_id in self.camera.robot_paths:
                            print(f"✔️ 로봇 {robot.robot_id}에게 경로가 계획되었습니다.")
                            print(f"    - 최종 목표: {goal_pos} 위치, {final_yaw} yaw")
                        else:
                            print(f"❌ 로봇 {robot.robot_id}의 경로를 찾을 수 없습니다.")

                elif task.robot_type == "MOBILE" and task.task_type == "MOVE_TO_RACK":
                    loc : MyLocation = task.location
                    
                    goal_pos = (loc.pose_m[0], loc.pose_m[1])  # task 객체에 goal_pos가 있다고 가정
                    final_yaw = loc.pose_m[2] # task 객체에 final_yaw가 있다고 가정

                    self.camera.set_robot_goal(
                        pos=None,
                        robot_id=robot.robot_id,
                        real_pos=goal_pos,
                        final_yaw=final_yaw
                    )
                    self.camera.plan_robot_paths()

                    if robot.robot_id in self.camera.robot_paths:
                        print(f"✔️ 로봇 {robot.robot_id}에게 경로가 계획되었습니다.")
                        print(f"    - 최종 목표: {goal_pos} 위치, {final_yaw} yaw")
                    else:
                        print(f"❌ 로봇 {robot.robot_id}의 경로를 찾을 수 없습니다.")

                elif task.robot_type == "MOBILE" and task.task_type == "WAIT_USER":
                    # 모바일 로봇이 사용자 대기 작업을 수행하는 경우
                    print(task.location)
                    robot.assign_task(task)
                    
                    time.sleep(5.0)
                    print("작업자 대기중 ...")
                    self.complete_task(robot.robot_id)
                    print("작업자 작업완료")

                elif task.robot_type == "ARM" and task.task_type == "LOAD":
                    loc : MyLocation = task.location
                    
                    goal_pos = (loc.pose_m[0], loc.pose_m[1])  # task 객체에 goal_pos가 있다고 가정
                    final_yaw = loc.pose_m[2] # task 객체에 final_yaw가 있다고 가정
                    
                    # task 토픽 발행
                    self.camera.ros_node.publish_task(robot.robot_id, "LOAD", x=goal_pos[0], y=goal_pos[1], yaw=final_yaw, pinky_id=process.assigned_mobile_robot_id)
                    print("robot arm !! publish task topic !!")
                    print(f"Robot4, 'LOAD', x={goal_pos[0]}, y={goal_pos[1]}, yaw={final_yaw}")

                elif task.robot_type == "ARM" and task.task_type == "UNLOAD":
                    loc : MyLocation = task.location
                    
                    goal_pos = (loc.pose_m[0], loc.pose_m[1])  # task 객체에 goal_pos가 있다고 가정
                    final_yaw = loc.pose_m[2] # task 객체에 final_yaw가 있다고 가정
                    
                    # task 토픽 발행
                    self.camera.ros_node.publish_task(robot.robot_id, "UNLOAD", x=goal_pos[0], y=goal_pos[1], yaw=final_yaw, pinky_id=process.assigned_mobile_robot_id)
                    messages.append(f"robot arm {robot.robot_id} 작업 지시")
                    print(f"Robot5, 'UNLOAD', x={goal_pos[0]}, y={goal_pos[1]}, yaw={final_yaw}")

                if callable(self.on_task_assigned):
                    try:
                        self.on_task_assigned(task, robot, process)
                    except Exception as e:
                        print(f"[TaskManager] plan hook error: {e}")

            else:
                messages.append(
                    f"⏸ No robot available for Task {task.task_id} ({task.robot_type})"
                )
                new_queue.put((task.priority, time.time(), task))

        self.task_queue = new_queue
        return "\n".join(messages)

    def select_robot(self, task: Task):
        closest = None
        min_dist = float("inf")
        for robot in self.robots.values():
            robot : Robot = robot
            if robot.is_available() and robot.robot_type == task.robot_type:
                if robot.robot_type == "MOBILE":
                    # 다른 process task에 이미 고정된 로봇인지 확인
                    if any(
                        c.assigned_mobile_robot_id == robot.robot_id and not c.is_done()
                        for c in self.all_process_tasks
                    ):
                        continue  # 이미 다른 process task에 묶여 있음
                dist = (robot.current_pose[0] - task.location.pose_m[0]) ** 2 + (
                    robot.current_pose[1] - task.location.pose_m[1]
                ) ** 2
                if dist < min_dist:
                    min_dist = dist
                    closest = robot
        return closest

    def complete_task(self, robot_id):
        robot : Robot = self.robots.get(robot_id)
        if not robot or not robot.current_task.task_type:
            return f"ℹ️ Robot {robot_id} has no active task."

        task : Task = robot.current_task
        loc : MyLocation = task.location
        if task.task_type == "WAIT_USER":
            self.camera.task_widget.visualizer_widget.set_location_occupied(loc.rack_id, loc.floor, loc.row, loc.col, True)
        task.complete_task()
        robot.status = "PENDING"
        robot.create_idle_task()

        # process task 찾고 다음 단계로 진행
        for comp in self.all_process_tasks:
            if task in comp.steps:
                comp.advance()
                next_step = comp.current_step()
                if next_step:
                    self.task_queue.put((next_step.priority, time.time(), next_step))
                    return f"✅ Completed task {task.task_id}, next step {next_step.task_type} enqueued"
                else:
                    return f"✅ Process task {comp.task_id} fully completed"

        return f"✅ Task {task.task_id} completed by Robot {robot_id}"


# -*- coding: utf-8 -*-
"""
'database.py' 모듈의 세션을 활용하여 로직을 수행하는 DBManager 클래스입니다.
"""
from sqlalchemy.orm import Session
from sqlalchemy import select, desc
from contextlib import contextmanager
from typing import List, Dict, Any, TypeVar, Type

# 'database.py'에 정의된 SessionLocal 및 Base를 직접 가져와 사용합니다.
import database
# 'models.py'에 정의된 모든 모델들을 불러옵니다.
from models import Rack, Location, Inventory, Inbound, Outbound, Orders, Orders_Item, Item

# Type hint를 위한 변수
T = TypeVar('T', bound='DBManager')

class DBManager:
    """
    미리 설정된 SQLAlchemy 세션을 사용하여 비즈니스 로직을 수행하는 싱글톤 클래스입니다.
    """
    _instance = None

    def __new__(cls: Type[T], *args, **kwargs) -> T:
        """
        클래스 인스턴스가 존재하지 않으면 새로 생성하고, 이미 존재하면 기존 인스턴스를 반환합니다.
        (싱글톤 패턴)
        """
        if cls._instance is None:
            cls._instance = super(DBManager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        """
        미리 정의된 database 모듈로부터 세션 팩토리와 Base를 가져와 초기화합니다.
        """
        if hasattr(self, 'initialized'):
            return
        
        self.SessionLocal = database.SessionLocal
        self.Base = database.Base
        self.initialized = True

    @contextmanager
    def get_session(self):
        """
        데이터베이스 세션을 제공하는 컨텍스트 매니저.
        'with' 문을 사용하여 세션을 안전하게 관리할 수 있습니다.
        """
        db = self.SessionLocal()
        try:
            yield db
        finally:
            db.close()

    def _get_next_available_location_id(self, db: Session) -> int | None:
        """
        Inventory에 등록되지 않은 location_id 중 가장 작은 값을 찾아서 반환합니다.
        (내부 사용)
        """
        subquery = select(Inventory.location_id)
        result = db.query(Location.location_id)\
            .filter(Location.location_id.notin_(subquery))\
            .order_by(Location.location_id.asc())\
            .first()

        return result[0] if result else None

    def _get_robot_coordinates(self, db: Session, location_id: int) -> dict[str, Any] | None:
        """
        주어진 location_id에 대한 로봇의 최종 x, y 좌표를 계산합니다.
        (내부 사용)
        """
        location_data = db.query(Location, Rack)\
            .join(Rack, Location.rack == Rack.rack)\
            .filter(Location.location_id == location_id)\
            .first()

        if not location_data:
            print(f"오류: Location ID {location_id}에 대한 정보를 찾을 수 없습니다.")
            return None

        location, rack = location_data
        y_col_offset = float(location.col_num - 1) * float(rack.cell_size)
        y_offset = 4
        y_coord = float(rack.y_start) + y_col_offset + y_offset

        x_diff = float(rack.x_start) - float(rack.x_end)
        x_offset = 16

        if x_diff < 0:
            x_coord = float(rack.x_start) - x_offset
            yaw = 180
        else:
            x_coord = float(rack.x_start) + x_offset
            yaw = 0
            
        return {
            "x": x_coord,
            "y": y_coord,
            "yaw": yaw,
            "rack": location.rack,
            "row_num": location.row_num,
            "col_num": location.col_num,
            "location_id": location.location_id
        }

    def find_next_robot_coordinates(self) -> dict[str, Any] | None:
        """
        다음 입고할 위치와 해당 로봇 좌표를 한 번에 계산하여 반환합니다.
        """
        with self.get_session() as db:
            next_location_id = self._get_next_available_location_id(db)
            if not next_location_id:
                print("📦 현재 입고 가능한 빈 위치가 없습니다.")
                return None
            return self._get_robot_coordinates(db, next_location_id)

    def get_outbound_history(self) -> List[Dict[str, Any]]:
        """
        모든 출고 내역과 각 출고에 포함된 상품 목록을 조회합니다.
        """
        with self.get_session() as db:
            try:
                query = select(
                    Outbound,
                    Orders_Item,
                    Item
                ).join(
                    Orders, Outbound.order_relation
                ).join(
                    Orders_Item, Orders.order_items
                ).join(
                    Item, Orders_Item.item_relation
                ).order_by(
                    Outbound.ob_dttm.desc()
                )

                result_rows = db.execute(query).all()

                outbound_data = {}
                for outbound_obj, order_item_obj, item_obj in result_rows:
                    ob_id = outbound_obj.ob_id
                    
                    if ob_id not in outbound_data:
                        outbound_data[ob_id] = {
                            "ob_id": outbound_obj.ob_id,
                            "ob_dttm": outbound_obj.ob_dttm.isoformat(),
                            "ob_status": outbound_obj.ob_status,
                            "order_id": outbound_obj.order_id,
                            "items": []
                        }
                    
                    outbound_data[ob_id]["items"].append({
                        "item_id": item_obj.item_id,
                        "item_name": item_obj.item_name,
                        "order_amount": order_item_obj.order_amount,
                        "unit_price": float(order_item_obj.unit_price)
                    })

                return list(outbound_data.values())

            except Exception as e:
                print(f"⛔ 출고 내역 조회 중 오류 발생: {e}")
                return []
    
# ----------------------------------------------------------------------------
# Workers (ROS / Camera / DB)
# ----------------------------------------------------------------------------
class RosWorker(QObject):
    log = Signal(str)
    heartbeat = Signal(float)

    def __init__(self):
        super().__init__()
        self.exec = SingleThreadedExecutor()
        self._running = True

    def add_node(self, node):
        self.exec.add_node(node)

    def remove_node(self, node):
        self.exec.remove_node(node)

    @Slot()
    def start(self):
        import time

        last = time.perf_counter()
        while self._running:
            self.exec.spin_once(timeout_sec=0.01)
            now = time.perf_counter()
            self.heartbeat.emit((now - last) * 1000.0)
            last = now

    @Slot()
    def stop(self):
        self._running = False
        try:
            self.exec.shutdown()
        except Exception:
            pass

# 2) DBWatcher: Qt 타이머로 폴링 + 변경 감지 + 시그널
from PySide6.QtCore import QObject, QTimer, Signal
from datetime import datetime


from sqlalchemy.orm import Session
from sqlalchemy import select, desc
from datetime import datetime
from typing import List, Dict, Any

# 수정된 DBManager 클래스와 Inbound 모델을 import 합니다.
# 이 파일이 db_manager.py 및 models.py와 같은 디렉토리에 있다고 가정합니다.
from models import Inbound, Outbound, Orders_Item, Item

# DBManager 인스턴스 생성 (싱글톤이므로 한 번만 생성)
# DBManager.__init__이 이제 인자를 받지 않으므로 괄호 안에 인자를 넣지 않습니다.
db_manager = DBManager()

class DBWatcherWorker(QObject):
    inbound_updated = Signal(list)  # 새로 추가된 inbound 레코드 목록을 전달
    outbound_updated = Signal(list)
    started = Signal()
    stopped = Signal()

    def __init__(self, interval_ms=2000):
        super().__init__()
        # 외부에서 DBManager를 직접 주입받는 대신, 전역 인스턴스를 사용합니다.
        self.db = db_manager 
        self.timer = QTimer(self)
        self.timer.setInterval(interval_ms)
        self.timer.timeout.connect(self._poll_db)
        self.last_seen_ib_dttm = None
        self.last_seen_ob_dttm = None

    def start(self):
        self.timer.start()
        self.started.emit()

    def stop(self):
        self.timer.stop()
        self.stopped.emit()
    
    def _poll_db(self):
        """
        SQLAlchemy를 사용하여 입고(Inbound) 및 출고(Outbound) 테이블의 새로운 레코드를 확인합니다.
        """
        self._poll_inbound()
        self._poll_outbound()

    def _poll_inbound(self):
        """
        Inbound 테이블의 새로운 레코드를 확인합니다.
        """
        new_rows = []
        with self.db.get_session() as session:
            try:
                # 마지막으로 본 시각 이후의 레코드만 조회
                query = select(Inbound).order_by(desc(Inbound.ib_dttm))
                if self.last_seen_ib_dttm:
                    query = query.where(Inbound.ib_dttm > self.last_seen_ib_dttm)
                
                rows = session.execute(query).scalars().all()

                if rows:
                    # SQLAlchemy 객체를 딕셔너리로 변환하여 Signal에 전달
                    new_rows = [
                        {
                            "ib_id": row.ib_id,
                            "item_id": row.item_id,
                            "item_amount": row.item_amount,
                            "ib_amount": row.ib_amount,
                            "ib_status": row.ib_status,
                            "ib_dttm": row.ib_dttm.isoformat()
                        } 
                        for row in rows
                    ]
                    
                    # 마지막으로 본 시각을 업데이트
                    self.last_seen_ib_dttm = rows[0].ib_dttm.isoformat()
                    self.inbound_updated.emit(new_rows)
            
            except Exception as e:
                print(f"⛔ 입고 데이터 조회 중 오류 발생: {e}")

    def _poll_outbound(self):
        """
        Orders + Orders_Item 테이블에서 order_status=0 (출고대기) 인 주문만 확인합니다.
        """
        new_rows = []
        with self.db.get_session() as session:
            try:
                # 주문일시 기준 내림차순 정렬 + 출고대기(status=0)만
                query = (
                    select(Orders)
                    .where(Orders.order_status == 0)   # ✅ 상태 필터 추가
                    .order_by(desc(Orders.order_dttm))
                )
                
                if self.last_seen_ob_dttm:
                    query = query.where(Orders.order_dttm > self.last_seen_ob_dttm)
                
                rows = session.execute(query).scalars().all()

                if rows:
                    new_rows = []
                    for row in rows:
                        new_rows.append({
                            "order_id": row.order_id,
                            "customer_id": row.customer_id,
                            "order_status": row.order_status,
                            "order_dttm": row.order_dttm.isoformat(),
                            "destination": row.destination,
                            "items": [
                                {
                                    "item_id": item.item_id,
                                    "order_amount": item.order_amount,
                                    "unit_price": float(item.unit_price)  # DECIMAL → float 변환
                                }
                                for item in row.order_items
                            ]
                        })

                    # 최신 주문 시각 갱신
                    self.last_seen_ob_dttm = rows[0].order_dttm.isoformat()
                    self.outbound_updated.emit(new_rows)
            
            except Exception as e:
                print(f"⛔ 주문 데이터 조회 중 오류 발생: {e}")



# ======================================================================
# 경로 계획 관련 클래스들
# ======================================================================


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
        self.robot_goals: Dict[int, Tuple[float, float, float]] = {}
        self.safety_margin = 1  # 로봇 간 안전 거리 (그리드 셀 단위)

    def set_obstacles(self, obstacles: Set[Tuple[int, int]]):
        """장애물 위치 설정"""
        self.obstacles = obstacles.copy()

    def set_robot_goal(self, robot_id: int, goal: Tuple[float, float, float]):
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
        # directions 리스트에 (0, 0) 추가
        directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
            (0,0)
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
                pos = goal[:2]  # 기존 (row, col) 형태
                distance = self.euclidean_distance(position, pos)
                robot_priorities.append((distance, robot_id, position))

        robot_priorities.sort()  # 거리 순으로 정렬

        # print(robot_priorities)

        # 각 로봇에 대해 순차적으로 경로 계획
        for _, robot_id, position in robot_priorities:
            if robot_id in self.robot_goals:
                goal = self.robot_goals[robot_id]
                if goal is None:
                    goal = position
                goal = goal[:2]
                path = self.astar_pathfind(position, goal, exclude_robot=robot_id)
                if path:
                    planned_paths[robot_id] = path
                    self.robot_paths[robot_id] = path
                else:
                    # 경로를 찾을 수 없으면 현재 위치 유지
                    planned_paths[robot_id] = [position]
                    self.robot_paths[robot_id] = [position]

        return planned_paths
    
from dataclasses import dataclass
from typing import Optional

@dataclass
class Location:
    """
    물류 창고 내의 위치를 나타내는 클래스입니다.
    그리드 및 실제 로봇 좌표 정보를 모두 포함합니다.

    속성:
        location_id (str): 고유한 위치 식별자 (예: 'R1-A-1', 'INBOUND_AREA', 'OUTBOUND_AREA')
        rack (str): 랙 번호 (예: 'R1')
        row_num (int): 랙의 행 번호
        col_num (int): 랙의 열 번호
        x_coord (float): 위치의 x 좌표
        y_coord (float): 위치의 y 좌표
        yaw (float): 위치의 yaw 각도
    """
    location_id: str
    rack: str
    row_num: int
    col_num: int
    x_coord: float
    y_coord: float
    yaw: float
    
    def __str__(self) -> str:
        """
        Location 객체를 사람이 읽기 쉬운 문자열로 변환합니다.
        """
        return (f"Location(ID: {self.location_id}, Rack: {self.rack}, "
                f"Row: {self.row_num}, Col: {self.col_num}, "
                f"Coords: ({self.x_coord:.2f}, {self.y_coord:.2f}, Yaw: {self.yaw:.2f}))")

# ======================================================================
# 작업 관리 UI 위젯
# ======================================================================

class TaskManagerWidget(QWidget):
    """작업 관리 페이지 위젯"""

    def __init__(self, camera=None):
        super().__init__()
        self.camera: GridCameraWidget = camera
        self.manager: TaskManager = TaskManager(camera=self.camera)
        self._dispatched = set()
        self.counter = 1
        self.ib_cnt = 0
        self.ob_cnt = 0

        # 1. 랙 관리자와 시각화 위젯을 먼저 초기화합니다.
        self.rack_manager : RackManager = RackManager()
        # RACK_CONFIG 변수를 사용하여 랙 생성
        for rack_data in RACK_CONFIG:
            new_rack = MyRack(
                rack_id=rack_data['rack_id'],
                rows=rack_data['rows'],
                cols=rack_data['cols'],
                floors=rack_data['floors'],
                cell_width_m=rack_data['cell_width_m'],
                cell_height_m=rack_data['cell_height_m'],
                margin_m=rack_data['margin_m'],
                x_m=rack_data['x_m'],
                y_m=rack_data['y_m']
            )

            for floor, row, col in rack_data.get('initial_occupied', []):
                if floor < new_rack.floors and row < new_rack.rows and col < new_rack.cols:
                    new_rack.locations[floor][row][col] = True
            
            self.rack_manager.add_rack(new_rack)
        # 2. visualizer_widget을 초기화합니다.
        self.visualizer_widget = StockVisualizerWidget(self.rack_manager)
        
        # 3. 모든 필요한 객체가 준비된 후에 UI를 초기화합니다.
        self.init_ui()

        # 5. 타이머 관련 코드는 그대로 둡니다.
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_tables)
        self.update_timer.start(1000)


    def init_ui(self):
        """작업 관리 UI 초기화"""
        main_layout = QVBoxLayout()

        # --- 메인 콘텐츠 영역: 시각화 위젯과 테이블을 좌우로 분리 ---
        content_splitter = QSplitter(Qt.Horizontal)
        
        # 2. 로봇 상태 및 작업 현황 테이블 (오른쪽 패널)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # 로봇 상태 테이블
        robot_status_label = QLabel("🤖 로봇 상태:")
        right_layout.addWidget(robot_status_label)
        
        self.robot_status_table = QTableWidget()
        self.robot_status_table.setColumnCount(6)
        robot_headers = ["로봇 ID", "타입", "상태", "현재 작업", "현재 위치", "배터리"]
        self.robot_status_table.setHorizontalHeaderLabels(robot_headers)
        self.robot_status_table.setMaximumHeight(200)
        self.robot_status_table.setColumnWidth(4, 200)
        right_layout.addWidget(self.robot_status_table)
        
        # --- 버튼 및 로그 영역 (하단에 배치) ---
        # 작업 추가 버튼들
        button_layout = QHBoxLayout()
        left_button_widget = QWidget()
        left_button_layout = QVBoxLayout(left_button_widget)
        self.btn_add_inbound = QPushButton("📦 입고 작업 추가")
        self.btn_add_inbound.clicked.connect(self.test_add_inbound)
        self.btn_add_inbound.setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }")
        left_button_layout.addWidget(self.btn_add_inbound)
        
        self.btn_add_outbound = QPushButton("📤 출고 작업 추가")
        self.btn_add_outbound.clicked.connect(self.test_add_outbound)
        self.btn_add_outbound.setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }")
        left_button_layout.addWidget(self.btn_add_outbound)
        
        self.btn_assign = QPushButton("🚚 작업 할당 실행")
        self.btn_assign.clicked.connect(self.test_assign_tasks_waypoints)
        self.btn_assign.setStyleSheet("QPushButton { padding: 10px; font-size: 14px; background-color: #4CAF50; color: white; }")
        left_button_layout.addWidget(self.btn_assign)

        right_button_widget = QWidget()
        right_button_layout = QVBoxLayout(right_button_widget)
        # 로봇 완료 버튼들
        robot_layout = QVBoxLayout()
        robot_label = QLabel("로봇 작업 완료:")
        robot_layout.addWidget(robot_label)
        
        for robot_id in self.manager.robots.keys():
            btn = QPushButton(f"✅ 로봇 {robot_id}")
            btn.clicked.connect(lambda checked, rid=robot_id: self.complete_task(rid))
            btn.setStyleSheet("QPushButton { padding: 8px; }")
            robot_layout.addWidget(btn)
        right_button_layout.addLayout(robot_layout)
        
        button_layout.addWidget(left_button_widget)
        button_layout.addWidget(right_button_widget)
        
        right_layout.addLayout(button_layout)
        content_splitter.addWidget(self.visualizer_widget)
        content_splitter.addWidget(right_panel)
        content_splitter.setSizes([200, 800])  # 왼쪽 패널 넓게, 오른쪽 패널 좁게
        main_layout.addWidget(content_splitter)

        # 테이블 위젯
        # 전체 작업 현황 테이블
        table_label = QLabel("📊 전체 작업 현황:")
        main_layout.addWidget(table_label)

        self.table = QTableWidget()
        self.table.setColumnCount(7)
        headers = [
            "Process ID", "Task ID", "작업 타입", "로봇 타입",
            "할당 로봇", "작업 상태", "목표 위치",
        ]
        self.table.setHorizontalHeaderLabels(headers)
        main_layout.addWidget(self.table)
        
        

        # 로그 출력 영역
        self.log_label = QLabel("📋 작업 로그:")
        self.log_area = QListWidget()
        # self.log_area.setMaximumHeight(150)
        main_layout.addWidget(self.log_label)
        main_layout.addWidget(self.log_area)
        
        self.setLayout(main_layout)

        # 초기 테이블 업데이트
        self.update_tables()

    def add_log(self, message):
        """로그 메시지 추가"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_area.addItem(f"[{timestamp}] {message}")
        self.log_area.scrollToBottom()

    def test_add_inbound(self, ib_id, amount = 4):
        if ib_id == False:
            ib_id = self.ib_cnt

        """입고 작업 추가"""
        # DB 풀링
        
        # 입고 박스 추가
        batch_size = 4
        
        # 박스 개수를 배치사이즈만큼 프로세스 반복
        remaining = amount
        while remaining > 0:
            current_batch = min(batch_size, remaining)
            # 입고 프로세스 생성
            process_task = self.create_inbound_task(f"ib_{ib_id}")
            self.manager.add_process_task(process_task)
            self.add_log(f"➕ 입고 작업 ib_{ib_id} 추가됨")
            self.ib_cnt += 1                
            remaining -= current_batch
        self.update_tables()

    def test_add_outbound(self, ob_id, items):
        """출고 작업 추가"""
        print(f"🚚 출고 등록: 주문번호={ob_id}")

        # ✅ 만약 DB 조회 결과가 없으면 임시 테스트 데이터 사용
        if not items:
            print("⚠️ 주문 아이템이 없어 테스트용 데이터를 사용합니다.")
            items = [{"item_id": 999, "order_amount": 4, "unit_price": 0.0}]  # 더미 데이터

        for item in items:
            item_id = item['item_id']
            total_amount = item['order_amount']
            unit_price = item['unit_price']

            print(f"  - 상품 {item_id} | 수량={total_amount} | 단가={unit_price}")

            # 배치 사이즈 (기본 4개 단위)
            batch_size = 4
            remaining = total_amount

            while remaining > 0:
                current_batch = min(batch_size, remaining)
                # 출고 태스크 생성
                process = self.create_outbound_task(f"ob_{self.ob_cnt}", item_id=item_id, item_amount=current_batch)
                self.ob_cnt += 1
                self.manager.add_process_task(process)
                self.add_log(f"📤 출고 작업 {self.ob_cnt} 추가됨 " f"(상품 {item_id}, 수량 {current_batch})")
                remaining -= current_batch

            # UI 갱신
            self.update_tables()
            
    def create_inbound_task(self, process_id):
        """입고 작업 생성: 물건을 가져와서 진열하는 작업"""
        rack_location : MyLocation = self.visualizer_widget.get_inbound_target_pose()
        ib_location : MyLocation = MyLocation(-1, -1, -1, -1, (0.6,0.28, 0.0))
        arm_location : MyLocation = MyLocation(-1, -1, -1, -1, (0.6,0.28, 0.0))
        steps = [
            Task(f"{process_id}_1", "MOVE_TO_INBOUND", "MOBILE", ib_location), # 랙 위치
            Task(f"{process_id}_2", "LOAD", "ARM", arm_location), # 로봇 암이 픽업하는 위치; 안줘도 됨
            Task(f"{process_id}_3", "MOVE_TO_RACK", "MOBILE", rack_location),
            Task(f"{process_id}_4", "WAIT_USER", "MOBILE", rack_location), # 랙 위치
        ]
        return ProcessTask(process_id, steps)

    def create_outbound_task(self, process_id, item_id = None, item_amount = None):
        """입고 작업 생성: 물건을 가져와서 진열하는 작업"""
        rack_location : MyLocation = self.visualizer_widget.get_outbound_target_pose()
        ob_location : MyLocation = MyLocation(-1, -1, -1, -1, (0.6,0.78, 0.0))
        arm_location : MyLocation = MyLocation(-1, -1, -1, -1, (0.6,0.78, 0.0))
        steps = [
            Task(f"{process_id}_1", "MOVE_TO_RACK", "MOBILE", rack_location),
            Task(f"{process_id}_2", "WAIT_USER", "MOBILE", rack_location), # 랙 위치
            Task(f"{process_id}_3", "MOVE_TO_OUTBOUND", "MOBILE", ob_location), # 랙 위치
            Task(f"{process_id}_4", "UNLOAD", "ARM", arm_location), # 로봇 암이 픽업하는 위치; 안줘도 됨
        ]
        return ProcessTask(process_id, steps)

    def test_assign_tasks_waypoints(self):
        """자동 작업 할당 실행"""
        msg = self.manager.assign_tasks_waypoint()
        self.add_log("🚚 자동 작업 할당 실행됨")
        for line in msg.split("\n"):
            if line.strip():
                self.add_log(line)
        self.update_tables()

    def assign_tasks(self):
        """작업 할당 실행"""
        msg = self.manager.assign_tasks()
        self.add_log("🚚 작업 할당 실행됨")
        for line in msg.split("\n"):
            if line.strip():
                self.add_log(line)

        self.update_tables()

    def complete_task(self, robot_id):
        """로봇 작업 완료 처리"""
        msg = self.manager.complete_task(robot_id)
        self.add_log(f"로봇 {robot_id}: {msg}")
        self.update_tables()

    def update_tables(self):
        """테이블 업데이트"""
        # 작업 테이블 업데이트
        tasks = []
        for comp in self.manager.all_process_tasks:
            if comp.steps:
                for step in comp.steps:
                    tasks.append(step)
        # print(tasks)

        self.table.setRowCount(len(tasks))
        # 로봇 상태 테이블 업데이트
        robots = self.camera.robots

        for row, task in enumerate(tasks):
            task : Task = task
            process_id = next(
                (c.task_id for c in self.manager.all_process_tasks if task in c.steps),
                "",
            )
            self.table.setItem(row, 0, QTableWidgetItem(process_id))
            self.table.setItem(row, 1, QTableWidgetItem(str(task.task_id)))
            self.table.setItem(row, 2, QTableWidgetItem(task.task_type))
            self.table.setItem(row, 3, QTableWidgetItem(task.robot_type))
            self.table.setItem(row, 4, QTableWidgetItem(str(task.assigned_robot) if task.assigned_robot else "-"))
            self.table.setItem(row, 5, QTableWidgetItem(task.status))
            self.table.setItem(row, 6, QTableWidgetItem(str(task.location.pose_m)))
            # 상태에 따른 색상 설정
            if task.status == "COMPLETED":
                self.table.item(row, 5).setBackground(Qt.GlobalColor.green)
            elif (
                task.status == "ASSIGNED"
                or task.status == "INPROGRESS"
            ):
                self.table.item(row, 5).setBackground(Qt.GlobalColor.yellow)
            elif task.status == "PENDING":
                self.table.item(row, 5).setBackground(Qt.GlobalColor.lightGray)

        # print(robots)
        self.robot_status_table.setRowCount(len(robots))

        for row, robot in enumerate(robots.values()):
            robot : Robot = robot
            # print(f"Row: {row}, Robot ID: {robot.robot_id}")
            self.robot_status_table.setItem(row, 0, QTableWidgetItem(str(robot.robot_id)))
            self.robot_status_table.setItem(
                row, 1, QTableWidgetItem(robot.robot_type)
            )
            self.robot_status_table.setItem(
                row, 2, QTableWidgetItem(robot.status)
            )

            current_task = robot.current_task.task_type if robot.current_task and robot.current_task.task_type else "-"
            
            self.robot_status_table.setItem(row, 3, QTableWidgetItem(current_task))
            self.robot_status_table.setItem(
                row, 4, QTableWidgetItem(f"({robot.current_pose[0]:.2f}, {robot.current_pose[1]:.2f}, {robot.current_pose[2]:.2f})")
            )
            battery_val = f"{robot.battery:.1f}%"
            self.robot_status_table.setItem(
                row, 5, QTableWidgetItem(battery_val)
            )
            # 로봇 상태에 따른 색상
            if robot.status == RobotStatus.BUSY:
                self.robot_status_table.item(row, 2).setBackground(
                    Qt.GlobalColor.yellow
                )
            elif robot.status == RobotStatus.IDLE:
                self.robot_status_table.item(row, 2).setBackground(Qt.GlobalColor.green)


# ======================================================================
# 이미지 경로 저장
# ======================================================================

import os
from ament_index_python.packages import get_package_share_directory


def get_image_path():
    # bolt_fms 패키지의 share 디렉터리
    share_dir = get_package_share_directory("bolt_fms")
    # 설치된 리소스 경로 조합
    img_path = os.path.join(share_dir, "resources", "images", "test.png")
    return img_path


# ======================================================================
# 카메라 및 그리드 관리 위젯
# ======================================================================
class GridCameraWidget(QWidget):
    """그리드 카메라 페이지 위젯"""

    def __init__(self, robots : dict = None):
        super().__init__()

        self.robots = robots

        # ROS2 초기화 (UI 초기화 전에 수행)
        self.ros_node = None
        self.ros_sub_node = None
        self.task_widget : TaskManagerWidget = None
        # === 설정 값 ===
        self.camera_index = 0
        self.grid_rows = 5
        self.grid_cols = 10
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
        self.enable_apriltag = True
        self.apriltag_detections = []
        self.robot_positions = {}  # 로봇 위치 저장

        # === 경로 계획 관련 ===
        self.path_planner : MultiRobotPathPlanner = MultiRobotPathPlanner(self.grid_rows, self.grid_cols)
        self.robot_goals = {}  # 로봇 목표 위치
        self.robot_paths = {}  # 로봇 경로
        self.setting_goal_for_robot = None  # 목표 설정 모드
        self.enable_path_planning = True
        self._last_plan_log = {}

        # === 웨이포인트 관련 ===
        self.robot_waypoints = {}  # 로봇별 웨이포인트 리스트 {robot_id: [(x,y), ...]}
        self.robot_current_waypoint_index = {}  # 로봇별 현재 웨이포인트 인덱스
        self.waypoint_tolerance = 0.05  # 웨이포인트 도달 허용 오차 (미터)
        self.enable_robot_control = True  # 로봇 제어 활성화 여부
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

        # 로봇 작업 완료
        self.robot_check = QTimer()
        self.robot_check.timeout.connect(self.check_robot_complete)
        self.robot_check.start(30)  # 100ms마다 로봇 제어

    def check_robot_complete(self):
        for robot in self.robots.values():
            task : Task = robot.current_task
            if robot.status == "COMPLETE":
                self.task_widget.complete_task(robot.robot_id)
                print(f"Robot {robot.robot_id} completed task: {task.status}")
            
                
                
            
        self.robots[2].status

    def set_task_widget(self, widget):
        self.task_widget = widget

    def init_ui(self):
        """카메라 UI 초기화"""
        main_layout = QHBoxLayout()

        # === 왼쪽 패널: 컨트롤 ===
        left_panel = QVBoxLayout()

        # 그리드 크기 설정
        grid_settings = QGridLayout()
        grid_settings.addWidget(QLabel("행 수:"), 0, 0)
        self.rows_spinbox = QSpinBox()
        self.rows_spinbox.setRange(5, 40)
        self.rows_spinbox.setValue(self.grid_rows)
        self.rows_spinbox.valueChanged.connect(self.update_grid_size)
        grid_settings.addWidget(self.rows_spinbox, 0, 1)

        grid_settings.addWidget(QLabel("열 수:"), 1, 0)
        self.cols_spinbox = QSpinBox()
        self.cols_spinbox.setRange(5, 50)
        self.cols_spinbox.setValue(self.grid_cols)
        self.cols_spinbox.valueChanged.connect(self.update_grid_size)
        grid_settings.addWidget(self.cols_spinbox, 1, 1)

        left_panel.addLayout(grid_settings)

        # 코너 설정 버튼
        self.corner_button = QPushButton("🎯 코너 4개 점 설정하기")
        self.corner_button.clicked.connect(self.start_corner_setting)
        left_panel.addWidget(self.corner_button)

        # 상태 표시
        self.status_label = QLabel("카메라 영상의 4개 코너를 순서대로 클릭하세요")
        self.status_label.setWordWrap(True)
        left_panel.addWidget(self.status_label)

        # 로봇 위치 리스트
        left_panel.addWidget(QLabel("🤖 로봇 위치:"))
        self.robot_list = QListWidget()
        left_panel.addWidget(self.robot_list)

        # 장애물 리스트
        left_panel.addWidget(QLabel("🚧 장애물 목록:"))
        self.obstacle_list = QListWidget()
        left_panel.addWidget(self.obstacle_list)

        # 웨이포인트 리스트
        left_panel.addWidget(QLabel("🎯 웨이포인트 목록:"))
        self.waypoint_list = QListWidget()
        left_panel.addWidget(self.waypoint_list)

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

        # # 웨이포인트 관련 UI
        # waypoint_layout = QHBoxLayout()
        # self.test_waypoints_button = QPushButton("테스트 웨이포인트 설정")
        # self.test_waypoints_button.clicked.connect(self.set_test_waypoints)
        # waypoint_layout.addWidget(self.test_waypoints_button)

        # self.clear_waypoints_button = QPushButton("웨이포인트 지우기")
        # self.clear_waypoints_button.clicked.connect(self.clear_waypoints)
        # waypoint_layout.addWidget(self.clear_waypoints_button)
        # path_layout.addLayout(waypoint_layout)

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

        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.image_label, 1)

        self.setLayout(main_layout)

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

    def init_ros2(self):
        """ROS2 초기화"""
        try:
            rclpy.init()
            self.ros_node = RobotStatusNode(robots=self.robots)
            if self.ros_status:
                self.ros_status.setText("ROS2: 연결됨")
            print("✅ ROS2 노드 초기화 완료")
        except Exception as e:
            if self.ros_status:
                self.ros_status.setText(f"ROS2: 오류 - {str(e)}")
            print(f"❌ ROS2 초기화 실패: {e}")
            self.ros_node = None

    def spin_ros(self):
        """ROS2 스핀"""
        if self.ros_node:
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.001)
            except Exception as e:
                print(f"ROS2 스핀 오류: {e}")

    # 나머지 메서드들 (원본과 동일)
    def update_frame(self):
        """프레임 업데이트"""
        ret, frame = self.camera.read()

        if self.camera is None or (
            hasattr(self.camera, "isOpened") and not self.camera.isOpened()
        ):
            frame = self._load_placeholder()
            self.current_frame = frame
            self.display_frame()
            # return

        self.current_frame = frame.copy()

        # AprilTag 감지
        if self.enable_apriltag:
            self.detect_apriltags()

        # # ROS2 토픽으로 발행
        # self.publish_robot_positions()

        # 그리드와 장애물 그리기
        self.draw_grid_and_obstacles()

        # Qt용 이미지로 변환
        self.display_frame()

    def _load_placeholder(self):
        """웹캠 미오픈/프레임 실패 시 보여줄 대체 이미지"""
        import cv2, os, numpy as np

        # 프로젝트 기준 절대경로로 읽는 게 안전합니다.
        img_path = get_image_path()
        # 예) OpenCV
        import cv2

        img = cv2.imread(img_path)
        if img is None:
            # 파일 없으면 검은 캔버스 생성 + 안내 문구
            w = getattr(self, "CAP_WIDTH", 1280)
            h = getattr(self, "CAP_HEIGHT", 720)
            img = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(
                img,
                "Webcam Off",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (255, 255, 255),
                3,
                cv2.LINE_AA,
            )
        return img

    def detect_apriltags(self):
        """AprilTag 감지 및 로봇 위치 추적"""
        if self.current_frame is None:
            return

        gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
        detections = self.apriltag_detector.detect(gray, estimate_tag_pose=False)

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
        # self.print_robot_positions()

        # ROS2 토픽으로 발행
        self.publish_robot_positions()

        # # 경로 업데이트
        # self.plan_robot_paths()

        # 웨이포인트 업데이트
        self.update_waypoint_following()

        self.update_robot_info()


        

    def update_robot_info(self):
        """
        로봇의 위치 정보를 기반으로 self.robots 객체의 current_pose를 업데이트합니다.
        """
        for robot_id in self.robot_positions.keys():
            robot_data = self.robot_positions.get(robot_id)
            if not robot_data: # 로봇 데이터가 없으면 건너뜀
                continue

            real_coords = robot_data.get("real_coords")
            yaw = robot_data.get("yaw")

            if not real_coords:
                continue

            robot = self.robots.get(robot_id)
            if not robot: # 로봇 객체가 없으면 건너뜀
                continue

            x, y = real_coords
            # ✅ 로봇 객체가 존재할 때만 포즈를 업데이트하도록 로직 수정
            robot.update_pose(x, y, yaw)
            # print(f"✅ 로봇 {robot_id}의 포즈 업데이트: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

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
                        f"🤖 로봇{robot_id} (태그{tag_id}): 그리드({col},{row}) 실제({real_x:.3f},{real_y:.3f}) yaw:{yaw:.3f}rad ({math.degrees(yaw):.1f}°)"
                    )
                elif grid_pos:
                    row, col = grid_pos
                    print(
                        f"🤖 로봇{robot_id} (태그{tag_id}): 그리드({col},{row}) yaw:{yaw:.3f}rad ({math.degrees(yaw):.1f}°) - 그리드 미설정"
                    )
                else:
                    print(
                        f"🤖 로봇{robot_id} (태그{tag_id}): yaw:{yaw:.3f}rad ({math.degrees(yaw):.1f}°) 그리드 범위 외부"
                    )

    # 간단한 메서드들만 포함 (공간 절약을 위해)
    def toggle_apriltag(self, state):
        """AprilTag 감지 활성화/비활성화"""
        self.enable_apriltag = state == Qt.CheckState.Checked.value
        if not self.enable_apriltag:
            self.apriltag_detections = []
            self.apriltag_status.setText("AprilTag: 비활성화")

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
        pt0, pt1 = corners[0], corners[1]
        dx = pt1[0] - pt0[0]
        dy = pt1[1] - pt0[1]
        # x축 기준 각도 계산
        yaw = -math.atan2(dy, dx)
        return yaw

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
                cv2.polylines(self.current_frame, [corners], True, color, 2)

                # 로봇 중심점 그리기 (크게)
                cv2.circle(self.current_frame, tuple(center), 4, color, -1)

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

    def draw_yaw_arrow(self, center, yaw, color):
        """로봇의 yaw 방향을 화살표로 표시"""
        arrow_length = 30
        arrow_end_x = int(center[0] + arrow_length * math.sin(yaw + math.pi / 2))
        arrow_end_y = int(center[1] + arrow_length * math.cos(yaw + math.pi / 2))
        # print(f"center{center} x {arrow_end_x} y {arrow_end_y}", )

        # 메인 화살표 선
        cv2.arrowedLine(
            self.current_frame,
            center,
            (arrow_end_x, arrow_end_y),
            (0, 0, 0),
            3,
            tipLength=0.3,
        )
        cv2.arrowedLine(
            self.current_frame,
            center,
            (arrow_end_x, arrow_end_y),
            color,
            2,
            tipLength=0.3,
        )

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
            [[0.0, 1.0], [1.0, 1.0], [0.0, 0.0], [1.0, 0.0]], dtype=np.float32
        )

        try:
            # 역변환 매트릭스
            matrix = cv2.getPerspectiveTransform(unit_square, corners)
            point = np.array([[[u, v]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(point, matrix)

            x, y = transformed[0][0]
            # print(f"real_coords_to_image_coords {x} {y}")
            return (x, y)
        except:
            return None

    def publish_path_as_waypoints(self, robot_id, path, final_yaw = 0.0):
        """A* 경로를 순차 웨이포인트로 변환하여 발행"""
        if not self.ros_node or len(path) <= 1:
            print(
                f"로봇 {robot_id} A* 경로 순차 웨이포인트 변환 실패!; no ros or no path"
            )
            return

        # 경로의 각 그리드 포인트를 실제 좌표로 변환
        path_waypoints = []
        for row, col in path[1:]:  # 현재 위치 제외
            real_coords = self.grid_to_real_coords(row, col)
            if real_coords:
                path_waypoints.append(real_coords)

        if not path_waypoints:
            return

        # # 첫 번째 경로 포인트를 즉시 발행
        # first_waypoint = path_waypoints[0]
        # self.ros_node.publish_target_pose(
        #     robot_id, first_waypoint[0], first_waypoint[1]
        # )
        # # 나머지 경로 포인트들을 로봇 경로 시스템에 저장
        # if len(path_waypoints) > 1:
        #     # 기존 웨이포인트를 경로 기반으로 업데이트
        #     remaining_waypoints = path_waypoints[1:]
        #     print(
        #         f"📍 로봇{robot_id} A* 경로 기반 웨이포인트 {len(remaining_waypoints)}개 추가 대기"
        #     )
        # # 목표 발행 상태 업데이트
        # self.robot_target_published[robot_id] = True

        # ✅ 웨이포인트 리스트 저장 + 인덱스 초기화 추가
        self.robot_waypoints[robot_id] = path_waypoints
        self.robot_current_waypoint_index[robot_id] = 0
        self.robot_target_published[robot_id] = False
        # self.update_waypoint_list()

        # 첫 번째 경로 포인트를 즉시 발행
        first_waypoint = path_waypoints[0]
        # second_waypoint = path_waypoints[1]
        # 현재 로봇의 실제 좌표가 있으면 방향 계산
        target_yaw = 0.0

        if len(path_waypoints) > 1:
            next_waypoint = path_waypoints[1]
            dx = next_waypoint[0] - first_waypoint[0]
            dy = next_waypoint[1] - first_waypoint[1]
            target_yaw = math.atan2(dy, dx)
            # if robot_id in self.robot_positions and self.robot_positions[robot_id].get(
            #     "real_coords"
            # ):
            #     rx, ry = self.robot_positions[robot_id]["real_coords"]
            #     dx = first_waypoint[0] - rx
            #     dy = first_waypoint[1] - ry
            #     target_yaw = math.atan2(dy, dx)
            print(
                f"웨이포인트{self.robot_current_waypoint_index[robot_id]}: x 좌표 {first_waypoint[0]},y 좌표 {first_waypoint[1]},yaw {target_yaw}"
            )
            self.ros_node.publish_target_pose(
                robot_id, first_waypoint[0], first_waypoint[1], final_yaw
            )
        else:
            self.ros_node.publish_target_pose(
                robot_id, first_waypoint[0], first_waypoint[1], target_yaw
            )
        self.robot_target_published[robot_id] = True

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
                # cv2.circle(self.current_frame, (int(x), int(y)), 10, wp_color, -1)
                cv2.circle(self.current_frame, (int(x), int(y)), 10, wp_color, 2)

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

    def grid_to_real_coords(self, row, col):
        """그리드 좌표를 실제 좌표로 변환"""
        real_x = (col + 0.5) * self.real_width / self.grid_cols
        real_y = (row + 0.5) * self.real_height / self.grid_rows
        # print(f"grid_to_real_coords {real_x} {real_y}")
        return (real_x, real_y)

    def draw_robot_goals(self):
        """로봇 목표 그리기"""
        if len(self.corner_points) < 4:
            return

        corners = np.array(self.corner_points[:4], dtype=np.float32)

        # ✅ 목표 정보를 (위치, 방향) 튜플로 받도록 수정
        for robot_id, goal in self.robot_goals.items():
            # goal에서 위치와 방향을 분리하여 사용
            row, col, yaw = goal

            color = self.get_robot_color(robot_id)

            # 목표 위치 계산
            cell_corners = []
            for r_offset, c_offset in [(0, 0), (0, 1), (1, 1), (1, 0)]:
                cell_row = (row + r_offset) / self.grid_rows
                cell_col = (col + c_offset) / self.grid_cols

                top_point = corners[2] + cell_col * (
                    corners[3] - corners[2]
                )  # 좌상, 우상
                bottom_point = corners[0] + cell_col * (
                    corners[1] - corners[0]
                )  # 좌하, 우하
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
            
            # ✅ 최종 포즈(방향)를 나타내는 선 그리기
            if yaw is not None:
                # 방향 벡터의 끝점 계산
                line_length = 20
                end_x = int(center[0] + line_length * math.cos(yaw))
                end_y = int(center[1] + line_length * math.sin(yaw))
                cv2.line(self.current_frame, tuple(center), (end_x, end_y), (255, 255, 255), 2)

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

                top_point = corners[2] + cell_col * (
                    corners[3] - corners[2]
                )  # 좌상, 우상
                bottom_point = corners[0] + cell_col * (
                    corners[1] - corners[0]
                )  # 좌하, 우하
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

        # 새로운 순서: 좌하, 우하, 좌상, 우상
        corners = np.array(self.corner_points[:4], dtype=np.float32)

        for row, col in self.obstacles:
            # 격자 셀의 4개 모서리 계산
            cell_corners = []
            for r_offset, c_offset in [(0, 0), (0, 1), (1, 1), (1, 0)]:
                cell_row = (row + r_offset) / self.grid_rows
                cell_col = (col + c_offset) / self.grid_cols

                # 새로운 순서에 맞게 인덱스 변경
                top_point = corners[2] + cell_col * (
                    corners[3] - corners[2]
                )  # 좌상, 우상
                bottom_point = corners[0] + cell_col * (
                    corners[1] - corners[0]
                )  # 좌하, 우하
                cell_point = top_point + cell_row * (bottom_point - top_point)
                cell_corners.append(cell_point.astype(int))
                # print(cell_corners)

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

    def set_robot_goal(self, pos, robot_id=None, real_pos=None, final_yaw=None):
        """
        로봇 목표 설정. 최종 포즈(위치와 방향)를 함께 저장합니다.

        Args:
            pos (tuple): 마우스 클릭으로 받은 이미지 좌표.
            robot_id (int): 목표를 설정할 로봇의 ID.
            real_pos (tuple): 실제 좌표 (x, y).
            final_yaw (float): 로봇의 최종 방향 (라디안).
        """
        if len(self.corner_points) < 4 or (self.setting_goal_for_robot is None and robot_id is None):
            return

        # Task 할당 시 실제 위치 설정
        if pos is None and real_pos is not None:
            grid_pos = self.real_coords_to_grid(real_x=real_pos[0], real_y=real_pos[1])
        else:
        # 마우스 클릭을 통해 이미지 pos 받을때
            grid_pos = self.pos_to_grid(pos)

        if grid_pos is None:
            return

        row, col = grid_pos
        if robot_id is None:
            robot_id = self.setting_goal_for_robot
            
        # 마우스 클릭으로 목표 설정 시 final_yaw가 없을 수 있으므로 기본값 0으로 설정
        if final_yaw is None:
            final_yaw = 0.0

        # ✅ 목표를 위치와 방향(yaw)을 포함한 튜플로 저장
        self.robot_goals[robot_id] = (row, col, final_yaw)
        self.path_planner.set_robot_goal(robot_id, (row, col, final_yaw))

        # UI 업데이트
        self.setting_goal_for_robot = None
        self.set_goal_button.setText("목표 설정")

        real_coords = self.grid_to_real_coords(row, col)
        if real_coords:
            real_x, real_y = real_coords
            print(
                f"🎯 로봇{robot_id} 목표 설정: 그리드({row},{col}) 실제({real_x:.2f},{real_y:.2f}), 최종 방향({math.degrees(final_yaw):.2f}도)"
            )
        return True

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
            # print(f"qt_to_image_coords {x} {y}")
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

    def update_obstacle_list(self):
        """장애물 리스트 UI 업데이트"""
        self.obstacle_list.clear()
        for row, col in sorted(self.obstacles):
            real_coords = self.grid_to_real_coords(row, col)
            if real_coords:
                real_x, real_y = real_coords
                item_text = f"그리드({row},{col}) → 실제({real_x:.2f},{real_y:.2f})"
                self.obstacle_list.addItem(item_text)

    def pos_to_grid(self, pos):
        """이미지 좌표를 그리드 좌표로 변환"""
        if len(self.corner_points) < 4:
            return None

        corners = np.array(self.corner_points[:4], dtype=np.float32)

        # 투시 변환으로 그리드 좌표 계산
        unit_square = np.array(
            [[0.0, 1.0], [1.0, 1.0], [0.0, 0.0], [1.0, 0.0]], dtype=np.float32
        )

        try:
            matrix = cv2.getPerspectiveTransform(corners, unit_square)
            point = np.array([[[float(pos[0]), float(pos[1])]]], dtype=np.float32)
            transformed = cv2.perspectiveTransform(point, matrix)

            u, v = transformed[0][0]
            row = int(v * self.grid_rows)
            col = int(u * self.grid_cols)
            # print(row, col)

            if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
                # print(f"pos_to_grid {row} {col}")
                return (row, col)
        except:
            pass

        return None

    def update_robot_list(self):
        """로봇 위치 리스트 UI 업데이트"""
        self.robot_list.clear()
        for robot_id in sorted(self.robot_positions.keys()):
            robot_data = self.robot_positions[robot_id]
            self.robot_list.addItem(
                f"로봇{robot_id}: 감지됨 태그{robot_data["tag_id"]}"
            )

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

        # print(f"real_coords_to_grid {grid_row} {grid_col}")

        return (grid_row, grid_col)

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

    def toggle_path_planning(self, state):
        """경로 계획 활성화/비활성화"""
        self.enable_path_planning = state == Qt.CheckState.Checked.value
        if not self.enable_path_planning:
            self.robot_paths = {}

    def start_goal_setting(self):
        """목표 설정 모드 시작"""
        robot_index = self.robot_goal_combo.currentIndex()
        robot_ids = list(ROBOT_CONFIG.keys())
        if robot_index < len(robot_ids):
            self.setting_goal_for_robot = robot_ids[robot_index]
            self.set_goal_button.setText(
                f"로봇{self.setting_goal_for_robot} 목표 설정 중..."
            )

    # def plan_robot_path(self):
    #     print(self.robot_positions)

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
        
    def clear_robot_path(self, robot_id):
        """특정 로봇의 경로 및 목표 지우기"""
        if robot_id in self.robot_paths:
            del self.robot_paths[robot_id]
            print(f"🗑️ 로봇 {robot_id}의 경로가 지워졌습니다.")
        
        if robot_id in self.robot_goals:
            del self.robot_goals[robot_id]
            print(f"🗑️ 로봇 {robot_id}의 목표가 지워졌습니다.")

        # 목표 설정 중인 로봇이 현재 로봇과 동일하다면 상태를 초기화
        if self.setting_goal_for_robot == robot_id:
            self.setting_goal_for_robot = None
            self.set_goal_button.setText("목표 설정")
        

    def clear_obstacles(self):
        """모든 장애물 제거"""
        self.obstacles.clear()
        self.update_obstacle_list()

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
        pass
        # if not self.robot_waypoints:
        #     print("⚠️ 설정된 웨이포인트가 없습니다.")
        #     return

        # if not self.enable_robot_control:
        #     print("⚠️ 먼저 로봇 자동 제어를 활성화하세요.")
        #     return

        # # 모든 로봇의 웨이포인트 인덱스를 처음으로 초기화
        # for robot_id in self.robot_waypoints.keys():
        #     self.robot_current_waypoint_index[robot_id] = 0
        #     self.robot_target_published[robot_id] = False

        # self.update_waypoint_list()
        # print("🚀 웨이포인트 미션을 시작합니다!")

        # # 첫 번째 웨이포인트들을 각 로봇에 발행
        # for robot_id, waypoints in self.robot_waypoints.items():
        #     if waypoints and robot_id in self.robot_positions:
        #         first_waypoint = waypoints[0]
        #         if self.ros_node:
        #             first_x, first_y = first_waypoint[0], first_waypoint[1]
        #             yaw0 = self._compute_yaw_for_target(robot_id, 0)
        #             self.ros_node.publish_target_pose(robot_id, first_x, first_y, yaw0)
        #             self.robot_target_published[robot_id] = True
        #             print(
        #                 f"📡 로봇{robot_id}에 첫 번째 웨이포인트 전송: {first_waypoint}"
        #             )

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

    def set_test_waypoints(self):
        pass
        # """테스트용 웨이포인트 설정"""
        # # 로봇 1에 대한 테스트 웨이포인트 (실제 좌표)
        # test_waypoints_robot1 = [
        #     (0.5, 0.2),  # 첫 번째 웨이포인트
        #     (1.2, 0.3),  # 두 번째 웨이포인트
        #     (1.5, 0.7),  # 세 번째 웨이포인트
        #     (0.8, 0.8),  # 마지막 웨이포인트
        # ]

        # # 로봇 2에 대한 테스트 웨이포인트 (실제 좌표)
        # test_waypoints_robot2 = [
        #     (1.8, 0.1),  # 첫 번째 웨이포인트
        #     (1.0, 0.4),  # 두 번째 웨이포인트
        #     (0.3, 0.6),  # 세 번째 웨이포인트
        # ]

        # if 1 in ROBOT_CONFIG:
        #     self.set_robot_waypoints(1, test_waypoints_robot1)
        # if 2 in ROBOT_CONFIG:
        #     self.set_robot_waypoints(2, test_waypoints_robot2)

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

    def clear_waypoints(self):
        """모든 웨이포인트 지우기"""
        self.robot_waypoints = {}
        self.robot_current_waypoint_index = {}
        self.robot_target_published = {}
        self.update_waypoint_list()
        print("🗑️ 모든 웨이포인트가 지워졌습니다.")

    def clear_robot_waypoints(self, robot_id):
        """특정 로봇의 웨이포인트 지우기"""
        if robot_id in self.robot_waypoints:
            del self.robot_waypoints[robot_id]
            print(f"🗑️ 로봇 {robot_id}의 웨이포인트가 지워졌습니다.")
        
        if robot_id in self.robot_current_waypoint_index:
            del self.robot_current_waypoint_index[robot_id]
            
        if robot_id in self.robot_target_published:
            del self.robot_target_published[robot_id]
            
        self.update_waypoint_list()

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
        # ✅ 딕셔너리의 사본(copy)을 순회하도록 수정
        for robot_id, waypoints in list(self.robot_waypoints.items()):
            if robot_id not in self.robot_positions:
                continue

            current_index = self.robot_current_waypoint_index.get(robot_id, 0)
            if current_index >= len(waypoints):
                continue

            # 현재 로봇 위치 (실제 좌표)
            robot_data = self.robot_positions[robot_id]
            real_coords = robot_data.get("real_coords")
            if not real_coords:
                continue

            robot_x, robot_y = real_coords
            # waypoints가 딕셔너리일 경우를 대비하여 수정
            if isinstance(waypoints, dict) and 'path' in waypoints:
                target_x, target_y = waypoints['path'][current_index]
            else:
                target_x, target_y = waypoints[current_index]

            # 목표까지의 거리 계산
            distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)

            if distance <= self.waypoint_tolerance:
                # 웨이포인트에 도달
                print(
                    f"✅ 로봇{robot_id} 웨이포인트 {current_index+1} 도달: ({target_x:.2f}, {target_y:.2f})"
                )
                
                self.robot_current_waypoint_index[robot_id] = current_index + 1
                
                # 다음 목표를 위해 리셋
                self.robot_target_published[robot_id] = False

                if current_index + 1 >= len(waypoints):
                    print(f"🏁 로봇{robot_id} 모든 웨이포인트 완료!")
                    # 모든 웨이포인트 완료 시 데이터 삭제
                    self.clear_robot_waypoints(robot_id)
                    self.clear_robot_path(robot_id)
                    
                    final_yaw = 0.0
                    if isinstance(waypoints, dict) and 'final_yaw' in waypoints:
                        final_yaw = waypoints['final_yaw']

                    final_x, final_y = waypoints[current_index]
                    
                    if self.enable_robot_control and self.ros_node:
                        print(f"✅ 로봇{robot_id} 최종 위치에서 원하는 포즈(yaw={final_yaw:.2f})로 회전")
                        self.ros_node.publish_target_pose(robot_id, final_x, final_y, final_yaw)
                        self.ros_node.stop_robot(robot_id)
                        self.task_widget.complete_task(robot_id)
                else:
                    # 다음 웨이포인트
                    if isinstance(waypoints, dict) and 'path' in waypoints:
                        next_waypoint = waypoints['path'][current_index + 1]
                    else:
                        next_waypoint = waypoints[current_index + 1]

                    if current_index + 2 < len(waypoints):
                        if isinstance(waypoints, dict) and 'path' in waypoints:
                            after_next = waypoints['path'][current_index + 2]
                        else:
                            after_next = waypoints[current_index + 2]
                        yaw_next = math.atan2(
                            after_next[1] - next_waypoint[1],
                            after_next[0] - next_waypoint[0],
                        )
                    else:
                        yaw_next = math.atan2(
                            next_waypoint[1] - robot_y, next_waypoint[0] - robot_x
                        )

                    print(
                        f"다음 웨이포인트를 발행 {robot_id, next_waypoint[0], next_waypoint[1], yaw_next}"
                    )
                    if self.enable_robot_control and self.ros_node:
                        self.ros_node.publish_target_pose(
                            robot_id, next_waypoint[0], next_waypoint[1], yaw_next
                        )
                        self.robot_target_published[robot_id] = True

                self.update_waypoint_list()
                self.draw_waypoints()

            else:
                if (
                    self.enable_robot_control
                    and self.ros_node
                    and not self.robot_target_published.get(robot_id, False)
                ):
                    yaw_cur = math.atan2(target_y - robot_y, target_x - robot_x)
                    print(
                        f"현재 웨이포인트를 발행 {robot_id,target_x,target_y,yaw_cur}"
                    )

                    self.ros_node.publish_target_pose(
                        robot_id, target_x, target_y, yaw_cur
                    )
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

            # ✅바뀐 경우에만 출력
            path_len = len(planned_paths[robot_id])
            key = (tuple(target_grid), path_len)
            if self._last_plan_log.get(robot_id) != key:
                print(f"🛣️ 로봇{robot_id} 웨이포인트 경로 계획: {path_len} 스텝")
                self._last_plan_log[robot_id] = key

            # A* 경로를 실제 웨이포인트로 변환하여 발행
            if self.enable_robot_control:
                self.publish_path_as_waypoints(robot_id, planned_paths[robot_id])
        else:
            # 경로 없음이면 이전 로그 상태 리셋(다음에 성공하면 다시 한 번만 로그)
            self._last_plan_log.pop(robot_id, None)
            print(f"⚠️ 로봇{robot_id} 웨이포인트 경로를 찾을 수 없습니다.")

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
            real_y = self.real_height - v * self.real_height

            if 0 <= u <= 1 and 0 <= v <= 1:
                # print(f"image_to_real_coords {real_x} {real_y}")
                return (real_x, real_y)
        except:
            pass

        return None


class InOutInventoryWidget(QWidget):
    """좌상=입고, 좌하=출고, 우측=재고 테이블 배치 탭"""

    def __init__(self, parent=None, task_widget=None):
        super().__init__(parent)

        # ── 테이블 위젯들 ─────────────────────────────────────
        self.inbound_table = QTableWidget(0, 4)  # 입고
        self.outbound_table = QTableWidget(0, 4)  # 출고
        self.inventory_table = QTableWidget(0, 5)  # 재고
        self.chatbot_widget = RAGChatbot(task_widget=task_widget)  # TaskManagerWidget 전달

        self.inbound_table.setHorizontalHeaderLabels(["입고ID", "품목", "수량", "일시"])
        self.outbound_table.setHorizontalHeaderLabels(
            ["출고ID", "품목", "수량", "일시"]
        )
        self.inventory_table.setHorizontalHeaderLabels(
            ["SKU", "품목", "재고수량", "위치", "최종갱신"]
        )

        # 더미데이터 채우기 (필요시 제거)
        self._fill_dummy()

        # ── 레이아웃: 좌측(상/하 스플리터) + 우측(재고) ───────
        left_splitter = QSplitter(Qt.Orientation.Vertical)
        left_splitter.addWidget(
            self._wrap_with_label("📥 입고 내역", self.inbound_table)
        )
        left_splitter.addWidget(
            self._wrap_with_label("📤 출고 내역", self.outbound_table)
        )
        left_splitter.setSizes([1, 1])  # ✅ 좌상/좌하 초기 높이 비율을 균일하게 설정

        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.addWidget(
            self._wrap_with_label("📦 재고 내역", self.inventory_table) # ✅ self.inventory_table로 수정
        )
        right_splitter.addWidget(
            self._wrap_with_label("❓ 물류 챗봇", self.chatbot_widget)
        )
        right_splitter.setSizes([1, 1])  # ✅ 재고/챗봇 초기 높이 비율을 균일하게 설정

        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_splitter.addWidget(left_splitter)
        main_splitter.addWidget(right_splitter)
        main_splitter.setSizes([1, 1])  # ✅ 좌/우 초기 폭 비율을 균일하게 설정

        # ── 탭 전체 레이아웃 ────────────────────────────────
        outer = QHBoxLayout(self)
        outer.addWidget(main_splitter)

    def _wrap_with_label(self, title: str, table: QTableWidget) -> QWidget:
        """섹션 제목 + 테이블을 세로로 감싸는 작은 패널"""
        w = QWidget()
        lay = QVBoxLayout(w)
        lbl = QLabel(title)
        lbl.setStyleSheet("font-weight: 600;")
        lay.addWidget(lbl)
        lay.addWidget(table)
        return w

    def _fill_dummy(self):
        """테스트용 더미 데이터"""
        inbound_rows = [
            ("IB-001", "모터 A", "10", "2025-08-10 10:05"),
            ("IB-002", "컨베이어벨트", "3", "2025-08-10 12:21"),
        ]
        outbound_rows = [
            ("OB-101", "모터 A", "4", "2025-08-10 13:10"),
            ("OB-102", "베어링 B", "6", "2025-08-10 13:32"),
        ]
        inventory_rows = [
            ("SKU-1001", "모터 A", "26", "A-01-03", "2025-08-10 13:10"),
            ("SKU-2002", "컨베이어벨트", "7", "B-02-01", "2025-08-10 12:21"),
            ("SKU-3003", "베어링 B", "14", "C-01-02", "2025-08-10 13:32"),
        ]

        def load_rows(table, rows):
            table.setRowCount(len(rows))
            for r, row in enumerate(rows):
                for c, val in enumerate(row):
                    table.setItem(r, c, QTableWidgetItem(val))
            table.resizeColumnsToContents()

        load_rows(self.inbound_table, inbound_rows)
        load_rows(self.outbound_table, outbound_rows)
        load_rows(self.inventory_table, inventory_rows)


class SingleButtonWidget(QWidget):
    def __init__(self, parent=None, camera=None):
        super().__init__(parent)
        self.camera = camera
        self.setWindowTitle("단일 버튼 위젯")

        # 버튼 하나
        self.button = QPushButton("눌러주세요")
        self.button.setMinimumHeight(40)
        self.button.clicked.connect(self.tasksimple)

        # 레이아웃(가운데 정렬)
        layout = QVBoxLayout(self)
        layout.addStretch()
        layout.addWidget(self.button, alignment=Qt.AlignCenter)
        layout.addStretch()
        self.setLayout(layout)

    def tasksimple(self):
        self.camera.ros_node.publish_task(5, "LOAD", x=0.58, y=0.28, yaw=0)
        print("robot arm !! publish task topic !!")
        print(f"5, 'LOAD', x=0.58, y=0.28, yaw=0")

    def task_unload(self):
        self.camera.ros_node.publish_task(5, "UNLOAD", x=0.58, y=0.58, yaw=0)
        print("robot arm !! publish task topic !!")
        print(f"5, 'LOAD', x=0.58, y=0.28, yaw=0")


# ======================================================================
# 메인 애플리케이션 클래스
# ======================================================================

class IntegratedGridCameraApp(QWidget):
    """통합 그리드 카메라 및 작업 관리 앱"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("🤖 통합 로봇 관리 시스템")
        self.setGeometry(0, 0, 1400, 900)
        self.db : DBManager = DBManager()

        self.robots = {}
        for robot_id, tag_id in ROBOT_CONFIG.items():
            if tag_id == 0:
                robot_type = "ARM" # 예시로 모든 로봇을 MOBILE로 설정
            else:
                robot_type = "MOBILE"

            self.robots[robot_id] = Robot(
                robot_id=robot_id,
                tag_id=tag_id,
                robot_type=robot_type,
                current_pose=(0,0,0),
                current_task= Task(-1,task_type="IDLE",robot_type=robot_type,location=None),
                status="PENDING",
                joint_angles=[]
            )
            print(f"로봇 객체 생성 완료: {self.robots[robot_id]}")
        self.init_ui()

        # ✅ DB Watcher (QThread)
        self.db_thread = QThread(self)
        self.db_watcher = DBWatcherWorker(interval_ms=2000)
        self.db_watcher.moveToThread(self.db_thread)
        # 스레드 수명/시작 연결
        self.db_thread.started.connect(self.db_watcher.start)
        self.db_watcher.stopped.connect(self.db_thread.quit)
        # 데이터 갱신 시그널 연결
        self.db_watcher.inbound_updated.connect(self.on_inbound_updated)
        self.db_watcher.outbound_updated.connect(self.on_outbound_updated)
        # 스레드 시작
        self.db_thread.start()

    def on_inbound_updated(self, rows):
        latest = max(rows, key=lambda x: datetime.fromisoformat(x["ib_dttm"]))
        ib_id = latest["ib_id"]
        amount = latest["ib_amount"]
        print(latest)

        self.task_widget.test_add_inbound(ib_id=ib_id, amount=amount)

        print(f"[Inbound] +{len(rows)} rows, latest ib_id={ib_id}, amount={amount}")
        
    def on_outbound_updated(self, rows):
        # 가장 최근 주문(출고) 선택
        latest = max(rows, key=lambda x: datetime.fromisoformat(x["order_dttm"]))
        
        order_id = latest["order_id"]
        customer_id = latest["customer_id"]
        order_status = latest["order_status"]
        order_dttm = latest["order_dttm"]

        # ✅ 주문 상품 dict 리스트 가져오기
        items_dict = latest["items"]

        print("📦 최신 주문:", latest)
        print("🛒 주문 상품 목록:", items_dict)

        self.task_widget.test_add_outbound(
            ob_id=order_id,  # 여기서는 order_id를 ob_id처럼 사용 가능
            items=items_dict
        )

        print(f"[Outbound] +{len(rows)} rows, latest order_id={order_id}, items={len(items_dict)}개")


    def init_ui(self):
        """메인 UI 초기화"""
        layout = QVBoxLayout()
        # 탭 위젯 생성
        self.tab_widget = QTabWidget()

        # 탭 1: 그리드 카메라 페이지
        self.camera_widget = GridCameraWidget(robots=self.robots)
        self.tab_widget.addTab(self.camera_widget, "📹 카메라 및 로봇 제어")

        # 탭 2: 작업 관리 페이지
        self.task_widget = TaskManagerWidget(
            # self.task_manager,
            camera=self.camera_widget
        )
        self.camera_widget.set_task_widget(self.task_widget)
        self.tab_widget.addTab(self.task_widget, "📋 작업 스케줄 관리")

        # 탭 3: 입출고 재고 내역 페이지
        self.table_widget = InOutInventoryWidget(task_widget=self.task_widget)
        self.tab_widget.addTab(self.table_widget, "📋 물류 입출고 재고 내역 확인")

        # 탭 4: 테스트
        self.test_widget = SingleButtonWidget(camera=self.camera_widget)
        self.tab_widget.addTab(self.test_widget, "📋 테스트")

        layout.addWidget(self.tab_widget)
        self.setLayout(layout)

    def closeEvent(self, event):
        """앱 종료시 정리 작업"""
        # 카메라 정리
        if hasattr(self.camera_widget, "camera") and self.camera_widget.camera:
            self.camera_widget.camera.release()
        cv2.destroyAllWindows()

        # ROS2 정리
        if hasattr(self.camera_widget, "ros_node") and self.camera_widget.ros_node:
            self.camera_widget.ros_node.destroy_node()
        try:
            rclpy.shutdown()
            self.db_watcher.stop()
        except:
            pass

        try:
            self.db_watcher.stop()
        except Exception as e:
            pass
        if self.db_thread.isRunning():
            self.db_thread.quit()
            self.db_thread.wait(1000)
        return super().closeEvent(e)
# ==============================================================================
# chatbot
# ==============================================================================
from sqlalchemy import create_engine, inspect, text
from sqlalchemy.orm import Session
import google.generativeai as genai
import pandas as pd
from database import Base, get_db
import markdown

# --- Gemini API 설정 ---
GOOGLE_API_KEY = "AIzaSyDBuxqZ-GN7TGk3gtZUTdSoETNd1KHBFEY"  # 실제 API 키로 변경하세요
if not GOOGLE_API_KEY:
    raise ValueError("GEMINI_API_KEY 환경 변수가 설정되지 않았습니다.")

genai.configure(api_key=GOOGLE_API_KEY)
MODEL_NAME = "gemini-1.5-flash-latest"

# --- SQLAlchemy 설정 ---
DATABASE_URL = "mysql+pymysql://admin:1q2w3e4r!@mydb1.ctmkoowy496q.ap-southeast-2.rds.amazonaws.com:3306/BoltDB"
engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,
    pool_recycle=3600,
)
SessionLocal = Session(bind=engine)

# --- 백그라운드 스레드: Gemini API 호출 ---
class GeminiWorker(QThread):
    finished = Signal(str)
    error = Signal(str)

    def __init__(self, prompt_text):
        super().__init__()
        self.prompt_text = prompt_text

    def run(self):
        try:
            model = genai.GenerativeModel(MODEL_NAME)
            response = model.generate_content(self.prompt_text)
            self.finished.emit(response.text)
        except Exception as e:
            self.error.emit(f"Gemini API 호출 중 오류 발생: {e}")

# --- DB 스키마 정보 가져오기 ---
def get_db_schema_info():
    # """DB의 모든 테이블 스키마 정보를 문자열로 반환합니다."""
    # inspector = inspect(engine)
    # table_names = inspector.get_table_names()

    # schema_info = "## 데이터베이스 테이블 스키마 정보\n\n"
    # for table_name in table_names:
    #     schema_info += f"### 테이블: {table_name}\n"
    #     columns = inspector.get_columns(table_name)
    #     schema_info += "| 컬럼명 | 데이터 타입 | Nullable |\n"
    #     schema_info += "|---|---|---|\n"
    #     for col in columns:
    #         schema_info += f"| {col['name']} | {col['type']} | {'Yes' if col['nullable'] else 'No'} |\n"
    #     schema_info += "\n"

    """제공된 상세한 DB 스키마 정보를 문자열로 반환합니다."""
    schema_info = """
    ## 데이터베이스 테이블 스키마 정보

    ### 1. Item (상품 설명)
    * **item_id**: INT, PRIMARY KEY, NOT NULL. 상품 고유번호. April tag 번호와 동일합니다.
    * **item_name**: VARCHAR(20), NOT NULL. 상품명.
    * **category**: VARCHAR(20), NOT NULL. 카테고리.
    * **price**: DECIMAL(10,2), NOT NULL. 단가.
    * **dimension_x**: DECIMAL(10,2), NOT NULL. 가로 길이.
    * **dimension_y**: DECIMAL(10,2), NOT NULL. 세로 길이.
    * **dimension_z**: DECIMAL(10,2), NOT NULL. 높이.

    ### 2. Rack (랙 위치)
    * **rack**: INT, PRIMARY KEY, NOT NULL. 랙 번호.
    * **x_start**: DECIMAL(10,2), NOT NULL. 랙의 x축 시작 좌표.
    * **x_end**: DECIMAL(10,2), NOT NULL. 랙의 x축 끝 좌표.
    * **y_start**: DECIMAL(10,2), NOT NULL. 랙의 y축 시작 좌표.
    * **y_end**: DECIMAL(10,2), NOT NULL. 랙의 y축 끝 좌표.
    * **cell_size**: INT, NOT NULL. 랙의 한 칸의 크기.

    ### 3. Location (적재 위치)
    * **location_id**: INT, PRIMARY KEY, AUTO_INCREMENT. 위치 고유번호.
    * **rack**: INT, NOT NULL, FOREIGN KEY. 랙 번호.
    * **row_num**: INT, NOT NULL. 선반 번호 (행).
    * **col_num**: INT, NOT NULL. 선반의 가로 위치 (열).
    * **description**: VARCHAR(100), NULL 허용. 위치에 대한 추가 설명.

    ### 4. Inventory (각 Item의 Location 정보)
    * **location_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 상품이 위치한 칸의 고유번호.
    * **item_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 상품 ID.
    * **iv_dttm**: DATETIME, NOT NULL. 재고가 쌓인 날짜와 시간.
    * **중요 규칙**: Inventory 테이블의 각 행은 재고에 있는 **상품 하나**를 의미합니다. 재고 수량은 이 테이블의 행 개수로 계산해야 합니다.

    ### 5. Orders (주문 정보)
    * **order_id**: INT, PRIMARY KEY, AUTO_INCREMENT. 주문 고유번호.
    * **customer_id**: INT, NOT NULL. 고객 고유번호.
    * **order_dttm**: DATETIME, NOT NULL. 주문일자 및 시간.
    * **order_status**: INT, NOT NULL. 주문 상태 (0: 집품대기, 1: 집품중, 2: 집품완료, 3: 출고대기).
    * **destination**: VARCHAR(40), NULL 허용. 배송지.

    ### 6. Orders_Item (각 주문별 상품)
    * **복합키**: (order_id, item_id)가 기본 키입니다.
    * **order_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 주문 ID.
    * **item_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 상품 ID.
    * **order_amount**: INT, NOT NULL. 주문 수량.
    * **unit_price**: DECIMAL(10,2), NOT NULL. 주문 당시 단가.

    ### 7. Outbound (출고 관련)
    * **order_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 주문 고유번호.
    * **ob_id**: INT, PRIMARY KEY, NOT NULL. 출고 바코드 번호 (포장 시 Apriltag 번호).
    * **ob_dttm**: DATETIME, NOT NULL. 출고일자 및 시간.
    * **ob_status**: INT, NOT NULL. 출고 상태 (0: 출고전, 1: 출고완료).
    * **destination**: VARCHAR(40), NULL 허용. 배송지.

    ### 8. Inbound (입고 관련)
    * **ib_id**: INT, PRIMARY KEY, AUTO_INCREMENT. 입고 ID.
    * **ib_amount**: INT, NOT NULL. 입고된 박스 수량.
    * **item_id**: INT, NOT NULL, FOREIGN KEY. 상품 ID.
    * **item_amount**: INT, NOT NULL. 한 박스에 들어있는 상품 개수.
    * **ib_dttm**: DATETIME, NOT NULL. 입고일자 및 시간.
    * **ib_status**: INT, NOT NULL. 입고 진행 상황.
    """
    return schema_info

# --- DB에서 실제 데이터 조회 ---
def get_db_data_for_rag(query_keywords):
    """
    모든 테이블의 데이터를 조회하여 문자열로 반환합니다.
    """
    related_data_info = ""
    with engine.connect() as connection:
        # models.py에 정의된 모든 테이블을 가져옵니다.
        tables = Base.metadata.tables
        
        for table_name, table in tables.items():
            try:
                sql_query = text(f"SELECT * FROM {table_name}")
                result = connection.execute(sql_query)
                df = pd.DataFrame(result.fetchall(), columns=result.keys())
                
                if not df.empty:
                    related_data_info += f"### 테이블 `{table_name}`의 전체 데이터\n"
                    related_data_info += df.to_markdown(index=False)
                    related_data_info += "\n\n"
            except Exception as e:
                print(f"Error fetching data from {table_name}: {e}")
                continue
                
    return related_data_info

class RAGChatbot(QWidget):
    def __init__(self, task_widget=None):
        super().__init__()
        self.task_widget = task_widget  # TaskManagerWidget 참조 저장
        self.initUI()
        self.db_schema = get_db_schema_info()

    def initUI(self):
        self.setWindowTitle('물류 창고 RAG 챗봇 (Gemini 1.5 Flash)')
        self.setGeometry(300, 300, 600, 500)

        main_layout = QVBoxLayout()

        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.chat_display.setStyleSheet("font-size: 14px; padding: 10px; background-color: #f0f0f0;")
        main_layout.addWidget(self.chat_display)

        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText('질문을 입력하세요...')
        self.input_field.returnPressed.connect(self.send_message)
        self.input_field.setStyleSheet("font-size: 14px; padding: 5px;")
        main_layout.addWidget(self.input_field)

        self.send_button = QPushButton('전송')
        self.send_button.clicked.connect(self.send_message)
        self.send_button.setStyleSheet("font-size: 14px; padding: 5px;")
        main_layout.addWidget(self.send_button)
        
        self.status_label = QLabel("준비 완료")
        self.status_label.setStyleSheet("font-size: 12px; color: gray;")
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)

    def send_message(self):
            user_question = self.input_field.text()
            if not user_question.strip():
                return
            
            self.chat_display.append(f"<p style='color: #007bff;'><b>나:</b> {user_question}</p>")
            self.input_field.clear()
            
            self.input_field.setEnabled(False)
            self.send_button.setEnabled(False)
            self.status_label.setText("데이터 조회 및 답변 생성 중...")

            # 1. 오늘 날짜 정보 가져오기
            today_date = datetime.now().strftime("%Y년 %m월 %d일")
            
            # 2-1. 키워드를 기반으로 DB에서 관련 데이터 조회
            db_data = get_db_data_for_rag(user_question)

            # 2-2. 로봇 상태 및 작업 현황 데이터를 문자열로 변환
            robot_status_info = ""
            task_status_info = ""
            
            if self.task_widget:
                robot_status_info = self.convert_table_to_string(self.task_widget.robot_status_table, "로봇 상태 현황")
                task_status_info = self.convert_table_to_string(self.task_widget.table, "전체 작업 현황")
            else:
                robot_status_info = "로봇 상태 데이터를 불러올 수 없습니다."
                task_status_info = "작업 현황 데이터를 불러올 수 없습니다."
            
            # 3. DB 스키마, 현재 날짜, 실제 데이터를 포함한 RAG 프롬프트 구성
            prompt_template = (
                "당신은 물류 창고 관리 시스템의 AI 챗봇입니다. "
                "사용자가 요청하는 데이터가 있을 경우, 아래 제공된 데이터베이스 스키마와 실제 데이터를 바탕으로 답변하세요. "
                "또한, 현재 **로봇 상태**와 **전체 작업 현황** 정보를 참고하여 답변을 보충하세요. "
                "특히, 아래 제공된 '메시지 해석 규칙'을 참고하여 로봇 ID, 작업 타입, 상태를 올바르게 해석하고, "
                "친절하고, 명확하며, 상세하게 설명해주세요. "
                "**아래에 제공된 '현재 날짜' 정보를 참고**하여 '오늘'이나 '어제'와 같은 시간 관련 질문에 답변해주세요."
                "데이터가 존재하지 않거나 질문에 대한 정보가 부족할 경우, '죄송합니다. 현재 데이터에 요청하신 정보가 없습니다.'와 같이 정중하게 답하고, "
                "필요한 정보를 구체적으로 알려주세요. "
                "답변은 가독성을 높이기 위해 불릿 포인트(`*` 또는 `-`)를 활용하여 정리해주세요.\n\n"
                
                f"{self.db_schema}\n\n"
                f"## 현재 날짜 정보\n"
                f"현재 날짜는 {today_date}입니다.\n\n"
                f"## 데이터베이스에서 조회한 실제 데이터\n"
                f"{db_data}"

                f"## 실시간 로봇 상태 및 작업 현황\n"
                f"{robot_status_info}\n"
                f"{task_status_info}\n\n"

                f"## 메시지 해석 규칙\n"
                f"- **로봇 ID 및 타입**: 각 로봇 ID는 다음과 같은 고유한 이름과 타입을 가집니다.\n"
                f"  - `1`: 볼트봇1 (모바일 로봇)\n"
                f"  - `2`: 볼트봇2 (모바일 로봇)\n"
                f"  - `3`: 볼트봇3 (모바일 로봇)\n"
                f"  - `4`: 볼트암1 (로봇팔)\n"
                f"  - `5`: 볼트암2 (로봇팔)\n\n"
                
                f"- **로봇 작업 (Task Type) 상세 설명**:\n"
                f"  - **모바일 로봇 (볼트봇 1, 2, 3)**: 바닥을 이동하며 물품을 운반하는 로봇입니다.\n"
                f"    - `IDLE`: 작업을 마치고 다음 명령을 대기하는 유휴 상태입니다.\n"
                f"    - `MOVE`: 특정 목표 위치로 이동 중입니다.\n"
                f"    - `MOVE_TO_INBOUND`: 입고 작업 물품을 받기 위해 '입고장'으로 이동 중입니다.\n"
                f"    - `MOVE_TO_OUTBOUND`: 출고 작업 물품을 전달하기 위해 '출고장'으로 이동 중입니다.\n"
                f"    - `CHARGE`: 배터리 충전을 위해 충전소로 이동하거나 충전 중입니다.\n"
                f"    - `WAIT_USER`: 작업자(사람)의 다음 지시를 기다리고 있습니다. 주로 랙(선반) 앞에서 집품/진열 작업을 대기할 때 사용됩니다.\n"
                f"  - **로봇 팔 (볼트암 4, 5)**: 특정 위치에서 물품을 집거나 놓는 작업을 수행합니다.\n"
                f"    - `IDLE`: 작업을 마치고 다음 명령을 대기하는 유휴 상태입니다.\n"
                f"    - `LOAD`: 입고 물품을 트레이나 다른 위치에 '적재'하고 있습니다.\n"
                f"    - `UNLOAD`: 트레이나 랙에 있는 물품을 '하역'하고 있습니다.\n\n"
                
                f"- **로봇 상태 (Status) 상세 설명**:\n"
                f"  - `PENDING`: 작업이 아직 할당되지 않고 대기 중인 상태입니다.\n"
                f"  - `ASSIGNED`: 작업이 할당되었지만 아직 시작하지 않은 상태입니다.\n"
                f"  - **모바일 로봇 (볼트봇)**:\n"
                f"    - `INPROGRESS`: 현재 할당된 `MOVE` 작업(이동)을 수행하고 있습니다.\n"
                f"    - `COMPLETE`: 할당된 작업을 완료했습니다.\n"
                f"    - `OBSTACLE`: 이동 경로에 장애물이 있어 멈춰선 상태입니다.\n"
                f"    - `LOW_BATTERY`: 배터리 잔량이 낮아 충전이 필요합니다.\n"
                f"  - **로봇 팔 (볼트암)**:\n"
                f"    - `INPROGRESS`: 현재 할당된 `PICK` 또는 `PLACE` 작업을 수행하고 있습니다.\n"
                f"    - `COMPLETE`: 할당된 작업을 완료했습니다.\n"
                f"    - `ERROR`: 작업 중 예상치 못한 오류가 발생했습니다.\n\n"
                
                f"- **작업 현황 (Process)**: 입고(`ib_`) 및 출고(`ob_`) 작업은 여러 개의 순차적인 작업(`Task ID`)으로 구성됩니다.\n"
                f"  - **입고 작업(`ib_`)**: `MOVE_TO_INBOUND` → `LOAD` → `MOVE_TO_RACK` → `WAIT_USER` 순서로 진행됩니다.\n"
                f"  - **출고 작업(`ob_`)**: `MOVE_TO_RACK` → `WAIT_USER` → `MOVE_TO_OUTBOUND` → `UNLOAD` 순서로 진행됩니다.\n\n"
                
                f"사용자 질문: {user_question}\n\n"
                "답변: "
            )


            # Gemini API 호출을 위한 스레드 시작
            self.gemini_worker = GeminiWorker(prompt_template)
            self.gemini_worker.finished.connect(self.handle_response)
            self.gemini_worker.error.connect(self.handle_error)
            self.gemini_worker.start()

    def handle_response(self, response_text):
        # Gemini의 마크다운 답변을 HTML로 변환
        html_response = markdown.markdown(response_text)
        
        # HTML을 `QTextEdit`에 추가 (append 대신 setHtml을 사용하면 기존 내용을 덮어씁니다)
        self.chat_display.append(f"<p><b>Gemini:</b> {html_response}</p>")
        self.reset_ui()

    def handle_error(self, error_message):
        self.chat_display.append(f"<p style='color: red;'><b>오류:</b> {error_message}</p>")
        self.reset_ui()

    
    def reset_ui(self):
        self.input_field.setEnabled(True)
        self.send_button.setEnabled(True)
        self.status_label.setText("준비 완료")

    # RAGChatbot 클래스에 새로운 메서드 추가
    def convert_table_to_string(self, table_widget, title):
        """QTableWidget의 내용을 Markdown 테이블 문자열로 변환합니다."""
        if not table_widget:
            return f"### {title}\n정보 없음.\n"

        data_list = []
        # 헤더 추출
        headers = [table_widget.horizontalHeaderItem(col).text() for col in range(table_widget.columnCount())]
        data_list.append(headers)

        # 데이터 추출
        for row in range(table_widget.rowCount()):
            row_data = [table_widget.item(row, col).text() for col in range(table_widget.columnCount())]
            data_list.append(row_data)

        # pandas DataFrame을 사용하여 Markdown으로 변환
        df = pd.DataFrame(data_list[1:], columns=data_list[0])
        markdown_table = df.to_markdown(index=False)
        
        return f"### {title}\n{markdown_table}\n"



class ClickableLabel(QLabel):
    """클릭 가능한 QLabel"""

    mouse_clicked = Signal(QPoint)

    def __init__(self):
        super().__init__()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.mouse_clicked.emit(event.position().toPoint())


def main():
    """메인 함수"""
    app = QApplication(sys.argv)

    # 앱 스타일 설정
    app.setStyle("Fusion")

    # 메인 윈도우 생성
    window = IntegratedGridCameraApp()
    window.show()

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("프로그램 종료")


if __name__ == "__main__":
    main()
