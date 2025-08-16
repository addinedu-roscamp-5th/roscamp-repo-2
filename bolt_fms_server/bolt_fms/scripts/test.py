# -*- coding: utf-8 -*-
"""
PySide6를 사용하여 랙의 층별 재고 상태를 그리드로 시각화하는 위젯입니다.
TaskManagerWidget에 StockVisualizerWidget이 통합되었습니다.
"""
import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGraphicsScene, QGraphicsView, 
    QComboBox, QLabel, QGraphicsRectItem, QGraphicsTextItem, QPushButton, QGraphicsEllipseItem, QGraphicsPolygonItem
)
from PySide6.QtCore import Qt, QPointF, QPoint, QRectF
from PySide6.QtGui import QBrush, QPen, QColor, QFont, QPolygonF
from typing import List, Dict, Tuple, Any, Optional
import math

from typing import NamedTuple, Tuple
from typing import Tuple

class Location:
    """
    로봇의 목표 포즈 정보를 담는 클래스입니다.
    """
    def __init__(self, rack_id: int, floor: int, row: int, col: int, pose_m: Tuple[float, float, float], direction: str):
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

class Rack:
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

class RackManager:
    def __init__(self):
        self.racks: Dict[int, Rack] = {}

    def add_rack(self, rack_obj: Rack):
        self.racks[rack_obj.rack_id] = rack_obj

    def get_rack(self, rack_id: int) -> Rack | None:
        return self.racks.get(rack_id)

    def get_all_racks(self) -> Dict[int, Rack]:
        return self.racks
    
    def is_location_available(self, rack_id: int, floor: int, row: int, col: int) -> bool:
        rack = self.racks.get(rack_id)
        if rack and floor < rack.floors and row < rack.rows and col < rack.cols:
            return not rack.locations[floor][row][col]
        return False

    def find_first_available_space(self) -> Optional[Tuple[int, int, int, int]]:
        sorted_racks = sorted(self.get_all_racks().values(), key=lambda r: r.rack_id)
        
        for rack in sorted_racks:
            for floor in range(rack.floors):
                for row in range(rack.rows):
                    for col in range(rack.cols):
                        if self.is_location_available(rack.rack_id, floor, row, col):
                            return (rack.rack_id, floor, row, col)
        
        return None

class StockVisualizerWidget(QWidget):
    def __init__(self, rack_manager: RackManager):
        super().__init__()
        self.rack_manager = rack_manager
        self.setWindowTitle("랙 재고 상태 시각화 (층별)")
        self.setGeometry(100, 100, 800, 600)
        
        self.SCALE_FACTOR = 50 # 1미터 = 50픽셀
        self.ROBOT_CLEARANCE_M = 0.4
        self.ROBOT_CLEARANCE_PX = self.ROBOT_CLEARANCE_M * self.SCALE_FACTOR
        self.ROBOT_RADIUS_PX = 15
        
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

        self.pose_label = QLabel("로봇 위치 (Pose): -")
        self.pose_label.setMinimumWidth(300)

        control_layout.addWidget(QLabel("층 선택:"))
        control_layout.addWidget(self.floor_selector)
        control_layout.addWidget(self.find_space_button)
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
                    f"물리적 위치: (X={self.robot_physical_pose_m.x():.2f}m, Y={self.robot_physical_pose_m.y():.2f}m)"
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
        우선순위: 오른쪽 > 왼쪽 > 아래 > 위
        """
        found_rack_obj = self.rack_manager.get_rack(target_rack_id)
        
        rack_boundaries = []
        for rack in self.rack_manager.get_all_racks().values():
            rack_width_px = rack.cols * (rack.cell_width_m * self.SCALE_FACTOR)
            rack_height_px = rack.rows * (rack.cell_height_m * self.SCALE_FACTOR)
            
            rack_rect = QRectF(
                rack.x_m * self.SCALE_FACTOR,
                rack.y_m * self.SCALE_FACTOR,
                rack_width_px,
                rack_height_px
            )
            rack_boundaries.append((rack.rack_id, rack_rect))
        
        cell_width_px = found_rack_obj.cell_width_m * self.SCALE_FACTOR
        cell_height_px = found_rack_obj.cell_height_m * self.SCALE_FACTOR
        
        # 랙의 절대 위치(픽셀) 계산
        rack_x_px = found_rack_obj.x_m * self.SCALE_FACTOR
        rack_y_px = found_rack_obj.y_m * self.SCALE_FACTOR
        
        candidate_poses = []
        
        # 오른쪽
        x_pos = rack_x_px + (col + 1) * cell_width_px + self.ROBOT_CLEARANCE_PX
        y_pos = rack_y_px + (found_rack_obj.rows - 1 - row) * cell_height_px + cell_height_px / 2
        candidate_poses.append((QPointF(x_pos, y_pos), 'right'))
        
        # 왼쪽
        x_pos = rack_x_px + col * cell_width_px - self.ROBOT_CLEARANCE_PX
        y_pos = rack_y_px + (found_rack_obj.rows - 1 - row) * cell_height_px + cell_height_px / 2
        candidate_poses.append((QPointF(x_pos, y_pos), 'left'))
        
        # 아래쪽
        x_pos = rack_x_px + col * cell_width_px + cell_width_px / 2
        y_pos = rack_y_px + (found_rack_obj.rows - 1 - (row - 1)) * cell_height_px + self.ROBOT_CLEARANCE_PX
        candidate_poses.append((QPointF(x_pos, y_pos), 'down'))

        # 위쪽
        x_pos = rack_x_px + col * cell_width_px + cell_width_px / 2
        y_pos = rack_y_px + (found_rack_obj.rows - 1 - row) * cell_height_px - self.ROBOT_CLEARANCE_PX
        candidate_poses.append((QPointF(x_pos, y_pos), 'up'))
        
        for pose, direction in candidate_poses:
            robot_rect = QRectF(pose.x() - self.ROBOT_RADIUS_PX, pose.y() - self.ROBOT_RADIUS_PX, self.ROBOT_RADIUS_PX * 2, self.ROBOT_RADIUS_PX * 2)
            is_safe = True
            for _, rack_rect in rack_boundaries:
                if robot_rect.intersects(rack_rect):
                    is_safe = False
                    break
            if is_safe:
                return pose, direction
        
        # 모든 경계가 다른 랙과 겹치는 경우, 통로 중앙으로 폴백합니다.
        aisle_width_px_fallback = found_rack_obj.margin_m * self.SCALE_FACTOR
        x_pos = rack_x_px + found_rack_obj.cols * cell_width_px + aisle_width_px_fallback / 2
        y_pos = rack_y_px + (found_rack_obj.rows - 1 - row) * cell_height_px + cell_height_px / 2
        return QPointF(x_pos, y_pos), 'left'

    def update_visualization(self):
        self.scene.clear()
        self.current_floor = self.floor_selector.currentIndex()
        
        font = QFont("Arial", 8)
        
        for rack in self.rack_manager.get_all_racks().values():
            if self.current_floor >= rack.floors:
                continue
            
            # 랙의 절대 위치(픽셀)
            rack_x_px = rack.x_m * self.SCALE_FACTOR
            rack_y_px = rack.y_m * self.SCALE_FACTOR
            
            # 랙의 전체 크기(픽셀)
            rack_width_px = rack.cols * (rack.cell_width_m * self.SCALE_FACTOR)
            rack_height_px = rack.rows * (rack.cell_height_m * self.SCALE_FACTOR)
            
            cell_width_px = rack.cell_width_m * self.SCALE_FACTOR
            cell_height_px = rack.cell_height_m * self.SCALE_FACTOR
            
            for r in range(rack.rows):
                for c in range(rack.cols):
                    grid_x = rack_x_px + c * cell_width_px
                    grid_y = rack_y_px + (rack.rows - 1 - r) * cell_height_px
                    
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
            robot_center = self.robot_physical_pose_px
            
            robot = QGraphicsEllipseItem(
                robot_center.x() - self.ROBOT_RADIUS_PX,
                robot_center.y() - self.ROBOT_RADIUS_PX,
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
                polygon.append(QPointF(robot_center.x() - self.ROBOT_RADIUS_PX, robot_center.y()))
                polygon.append(QPointF(robot_center.x() - self.ROBOT_RADIUS_PX - arrow_size, robot_center.y() - arrow_size / 2))
                polygon.append(QPointF(robot_center.x() - self.ROBOT_RADIUS_PX - arrow_size, robot_center.y() + arrow_size / 2))
            elif self.robot_direction == 'right':
                polygon.append(QPointF(robot_center.x() + self.ROBOT_RADIUS_PX, robot_center.y()))
                polygon.append(QPointF(robot_center.x() + self.ROBOT_RADIUS_PX + arrow_size, robot_center.y() - arrow_size / 2))
                polygon.append(QPointF(robot_center.x() + self.ROBOT_RADIUS_PX + arrow_size, robot_center.y() + arrow_size / 2))
            elif self.robot_direction == 'up':
                polygon.append(QPointF(robot_center.x(), robot_center.y() - self.ROBOT_RADIUS_PX))
                polygon.append(QPointF(robot_center.x() - arrow_size / 2, robot_center.y() - self.ROBOT_RADIUS_PX - arrow_size))
                polygon.append(QPointF(robot_center.x() + arrow_size / 2, robot_center.y() - self.ROBOT_RADIUS_PX - arrow_size))
            elif self.robot_direction == 'down':
                polygon.append(QPointF(robot_center.x(), robot_center.y() + self.ROBOT_RADIUS_PX))
                polygon.append(QPointF(robot_center.x() - arrow_size / 2, robot_center.y() + self.ROBOT_RADIUS_PX + arrow_size))
                polygon.append(QPointF(robot_center.x() + arrow_size / 2, robot_center.y() + self.ROBOT_RADIUS_PX + arrow_size))
            
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

    def get_robot_target_pose(self) -> Location:
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

        # 픽셀을 미터로 변환
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

        self.set_location_occupied(rack_id, floor, row, col, True)
        location = Location(rack_id=rack_id, floor=floor, row=row, col=col, pose_m=(x_m,y_m,yaw_rad),direction=robot_direction)
        return location

# racks_config.py 모듈에서 RACK_CONFIG 변수를 임포트
RACK_CONFIG = [
    {
        "rack_id": 1,
        "rows": 2,
        "cols": 2,
        "floors": 5,
        "cell_width_m": 1,
        "cell_height_m": 2,
        "margin_m": 1.5,
        "x_m": 1.0,
        "y_m": 0.3,
        "initial_occupied": [
        ]
    },
    {
        "rack_id": 2,
        "rows": 2,
        "cols": 2,
        "floors": 3,
        "cell_width_m": 0.6,
        "cell_height_m": 0.5,
        "margin_m": 0.3,
        "x_m": 5.0,
        "y_m": 0.5,
        "initial_occupied": [
        ]
    }
]

class TaskManagerWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Task Manager")
        self.setGeometry(100, 100, 1200, 800)
        
        self.rack_manager = RackManager()

        # RACK_CONFIG 변수를 사용하여 랙 생성
        for rack_data in RACK_CONFIG:
            new_rack = Rack(
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

        main_layout = QVBoxLayout(self)
        self.visualizer_widget = StockVisualizerWidget(self.rack_manager)
        main_layout.addWidget(self.visualizer_widget)

# ---
# 사용 예시: 이제 TaskManagerWidget을 실행합니다.
# ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # TaskManagerWidget을 직접 생성하고 실행
    window = TaskManagerWidget()
    window.show()
    
    sys.exit(app.exec())