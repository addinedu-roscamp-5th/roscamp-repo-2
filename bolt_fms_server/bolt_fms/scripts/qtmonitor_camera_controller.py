#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import atan2, sin, cos

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

# === ROS 노드 클래스 ===
class PoseMultiPublisher(Node):
    def __init__(self, robot_ids, get_pose_func_map, publish_period=0.03):
        super().__init__("multi_pose_publisher")
        self.pose_publishers = {}  # {id: publisher}
        self.get_pose_func_map = get_pose_func_map  # {id: func}

        for robot_id in robot_ids:
            topic = f"/robot{robot_id}/camera_pose"
            self.pose_publishers[robot_id] = self.create_publisher(
                PoseStamped, topic, 10
            )
            self.create_timer(
                publish_period, lambda rid=robot_id: self.publish_pose(rid)
            )

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
        self.get_logger().info(
            f"📤 /robot{robot_id}/camera_pose → x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

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
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QSizePolicy, QListWidget, QListWidgetItem, QLineEdit
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, Signal

# 📌 별도 정의된 ImageLabel 클래스 필요 (mouse_moved 시그널 포함)
# 📌 ImageWindow 사용 전 정의돼야 함


class ImageWindow(QWidget):
    def __init__(self, robot_tag_map):
        super().__init__()
        self.setWindowTitle("AprilTag Viewer")
        self.setGeometry(100, 100, 1280, 720)

        self.robot_tag_map = robot_tag_map
        self.selected_robot_id = None
        self.cursor_pos = None
        self.webcam_thread = None
        self.current_frame = None

        

        # ▶ 로봇 리스트 (왼쪽 위)
        self.robot_list = QListWidget()
        self.robot_list.setFixedWidth(200)
        self.robot_list.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self.robot_list.itemClicked.connect(self.handle_robot_select)

        for robot_id in self.robot_tag_map:
            item = QListWidgetItem(f"🤖 로봇-{robot_id}")
            self.robot_list.addItem(item)

        robot_list = QVBoxLayout()
        robot_list.addWidget(self.robot_list)

        # ▶ 태그 수정 폼
        self.tag_info_label = QLabel("태그 ID:")
        self.tag_edit = QLineEdit()
        self.tag_edit.setFixedWidth(100)
        self.save_button = QPushButton("💾 저장")
        self.save_button.setFixedWidth(100)
        self.save_button.clicked.connect(self.save_tag_id)
        
        self.tag_x_label = QLabel("X :")
        self.tag_x = QLineEdit()
        self.tag_x.setFixedWidth(100)
        self.tag_y_label = QLabel("Y :")
        self.tag_y = QLineEdit()
        self.tag_y.setFixedWidth(100)
        self.tag_yaw_label = QLabel("Yaw :")
        self.tag_yaw = QLineEdit()
        self.tag_yaw.setFixedWidth(100)

        tag_form = QVBoxLayout()
        tag_form.addWidget(self.tag_info_label)
        tag_form.addWidget(self.tag_edit)
        tag_form.addWidget(self.save_button)

        # ▶ 리셋 버튼
        self.refresh_button = QPushButton("🔄 리셋")
        self.refresh_button.setFixedSize(100, 40)
        self.refresh_button.clicked.connect(self.handle_refresh_click)
        # ▶ 오른쪽 레이아웃

        top_layout = QHBoxLayout()
        top_layout.addLayout(tag_form)
        top_layout.addStretch()
        top_layout.addWidget(self.refresh_button)
        
        # ▶ 오른쪽 위젯
        top_widget = QWidget()
        top_widget.setLayout(top_layout)

        tag_x_form = QHBoxLayout()
        tag_x_form.addWidget(self.tag_x_label)
        tag_x_form.addWidget(self.tag_x)
        tag_y_form = QHBoxLayout()
        tag_y_form.addWidget(self.tag_y_label)
        tag_y_form.addWidget(self.tag_y)
        tag_yaw_form = QHBoxLayout()
        tag_yaw_form.addWidget(self.tag_yaw_label)
        tag_yaw_form.addWidget(self.tag_yaw)

        left_bottom_layout = QVBoxLayout()
        left_bottom_layout.addLayout(tag_form)
        left_bottom_layout.addLayout(tag_x_form)
        left_bottom_layout.addLayout(tag_y_form)
        left_bottom_layout.addLayout(tag_yaw_form)

        left_layout = QVBoxLayout()
        left_layout.addLayout(robot_list)      
        # left_layout.addStretch()
        left_layout.addLayout(left_bottom_layout)   # 🔽 아래: 태그 폼
        # left_layout.addSpacing(10)

        left_widget = QWidget()
        left_widget.setLayout(left_layout)

        # ▶ 이미지 레이블
        self.image_label = ImageLabel(self)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        size_policy = QSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.image_label.setSizePolicy(size_policy)
        self.image_label.mouse_moved.connect(self.handle_mouse_move)
        self.image_label.mouse_clicked.connect(self.handle_mouse_click)


        # ▶ 중앙 레이아웃 (이미지 + 오른쪽 버튼)
        center_layout = QVBoxLayout()
        center_layout.addWidget(top_widget)  # ✅ 오른쪽에 리셋 버튼 위치 고정
        center_layout.addWidget(self.image_label, stretch=1)

        # ▶ 전체 레이아웃
        main_layout = QHBoxLayout()
        main_layout.addWidget(left_widget)     # 왼쪽: 로봇 리스트 + 태그폼
        main_layout.addLayout(center_layout, stretch=1)  # 중앙: 영상 + 오른쪽 버튼
        self.setLayout(main_layout)

    def handle_mouse_click(self):
        if self.webcam_thread.grid_corners is None:
            print("⚠️ 이미지 또는 그리드 코너 정보 없음")
            return

        x, y = self.cursor_pos[0], self.cursor_pos[1]
        print(f"🖱️ 마우스 클릭 위치: ({x}, {y})")

        real_x, real_y = self.webcam_thread.transed_point(x, y)

        if 0<= real_x <=REAL_MAX_WIDTH and 0<=real_y<=REAL_MAX_HEIGHT:
            yaw = 0.0  # 아직 yaw는 계산 안 했으므로 기본값 또는 나중에 보정
            print(f"🖱️ 변환된 클릭 위치: ({real_x}, {real_y})")

        return real_x, real_y

    # ▶ 로봇 선택 시 태그 에디터 업데이트
    def handle_robot_select(self, item):
        text = item.text()
        if "로봇-" in text:
            robot_id = int(text.split("-")[1])
            self.selected_robot_id = robot_id
            tag_id = self.robot_tag_map.get(robot_id, "")
            pose = self.webcam_thread.get_tag_pose(tag_id)[0]
            self.tag_edit.setText(str(tag_id))
            self.tag_x.setText(str(pose[0]))
            self.tag_y.setText(str(pose[1]))
            self.tag_yaw.setText(str(pose[2]))

    # ▶ 태그 ID 저장
    def save_tag_id(self):
        if self.selected_robot_id is not None:
            new_id = self.tag_edit.text()
            if new_id.isdigit():
                self.robot_tag_map[self.selected_robot_id] = int(new_id)
                print(f"[✅] 로봇-{self.selected_robot_id} → 태그 ID: {new_id} 저장됨")

    def handle_mouse_move(self, pos: QPoint):
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

    # 리셋 버튼 클릭 시 실행될 안전한 콜백 함수 정의
    def handle_refresh_click(self):
        if self.webcam_thread is not None and self.current_frame is not None:
            self.webcam_thread.redraw_grid_corners(self.current_frame)
            self.refresh_image()  # 👈 UI도 갱신

    def set_webcam_thread(self, thread):
        self.webcam_thread = thread

    def update_image(self, frame):
        self.current_frame = frame
        self.refresh_image()

    def resizeEvent(self, event):
        self.refresh_image()
        return super().resizeEvent(event)

    def refresh_image(self):
        if self.current_frame is None:
            return

        rgb_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)

        # 커서 위치 표시
        if self.cursor_pos is not None:
            x, y = self.cursor_pos
            cv2.putText(
                rgb_image,
                f"({x}, {y})",
                (x + 10, y + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2,
            )
            cv2.circle(rgb_image, (x, y), 5, (0, 0, 0), 2)


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

    # 스레드 실행 시 콜백 전달
    webcam_thread = WebcamThread(
        tag_id=TAG_ID,
        camera_index=CAMERA_INDEX,
        visualize=True,
        frame_callback=image_window.update_image,
    )
    webcam_thread.start()

    image_window.set_webcam_thread(webcam_thread)

    rclpy.init(args=None)

    robot_ids = list(robot_tag_map.keys())

    # pose 함수 매핑 생성
    def make_pose_func(tag_id):
        return lambda: webcam_thread.get_transed_pose_for_tag(tag_id)

    get_pose_func_map = {rid: make_pose_func(rid) for rid in robot_ids}

    # 노드 실행
    multi_publisher_node = PoseMultiPublisher(
        robot_ids, get_pose_func_map, publish_period=1
    )

    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(multi_publisher_node), daemon=True
    )
    ros_thread.start()

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
