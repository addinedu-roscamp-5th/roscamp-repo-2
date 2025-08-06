#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import atan2, sin, cos
from pupil_apriltags import Detector
import cv2
import numpy as np

from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QWidget,
    QVBoxLayout,
    QPushButton,
    QHBoxLayout,
    QTextEdit,
    QLineEdit,
)
from PySide6.QtGui import QImage, QPixmap, QMouseEvent
from PySide6.QtCore import QTimer

import sys

print("---------------------------------------------------------------")
print(sys.executable)  # 실행 중인 파이썬 경로 확인


from pupil_apriltags import Detector

# === 설정 ===
TAG_ID = 2
CAMERA_INDEX = 2
VISUALIZE = True

# === 그리드 설정 ===
CAP_RATIO = 1

CAP_WIDTH = int(1920 * CAP_RATIO)
CAP_HEIGHT = int(1080 * CAP_RATIO)

TOP_LEFT = (0, 0)
TOP_RIGHT = (CAP_WIDTH - 1, 0)
BOT_LEFT = (0, CAP_HEIGHT - 1)
BOT_RIGHT = (CAP_WIDTH - 1, CAP_HEIGHT - 1)

ROWS, COLS = 5, 10
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


class TargetPosePublisher(Node):
    def __init__(self, robot_id, tag_id):
        super().__init__(f"robot{robot_id}_target_publisher")
        self.robot_id = robot_id
        self.tag_id = tag_id
        self.publisher = self.create_publisher(
            PoseStamped, f"/robot{robot_id}/target_pose", 10
        )

    def publish_target_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"📤 /robot{self.robot_id}/target_pose → x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
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
                current_tag_poses_list.append(
                    {
                        "id": tag.tag_id,
                        "pose": (cX, cY, -yaw),
                        "corners": [(int(p[0]), int(p[1])) for p in tag.corners],
                    }
                )
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


from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
)
from PySide6.QtCore import Qt


class ImageWindow(QWidget):
    def __init__(self, robot_tag_map=None, webcam_thread=None):
        super().__init__()
        self.setWindowTitle("AprilTag Viewer (Qt)")
        self.robot_tag_map = robot_tag_map or {}
        self.webcam_thread = webcam_thread
        self.current_frame = None
        self.draw_grid_once = True
        # 타이머 설정 (0.5초 간격)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_selected_pose)
        self.update_timer.start(500)
        self.selected_row = None  # 선택된 행 번호

        # 이미지 및 Reset 버튼
        self.image_label = QLabel()
        self.image_label.setScaledContents(True)  # 이미지를 라벨 크기에 맞게 스케일 조정
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_grid_corners)

        # 하단: 테이블과 위치 출력창
        self.robot_table = QTableWidget()
        self.robot_table.setColumnCount(2)
        self.robot_table.setHorizontalHeaderLabels(["Robot ID", "Tag ID"])
        self.robot_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.robot_table.verticalHeader().setVisible(False)
        self.robot_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.robot_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.robot_table.setSelectionMode(QTableWidget.SingleSelection)
        self.robot_table.cellClicked.connect(self.on_table_click)

        self.pose_label = QLabel("🛰️ 위치 정보: 없음")
        self.pose_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.pose_label.setStyleSheet("font-size: 14px; color: #003366")

        # 전체 레이아웃 구성
        main_layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        top_layout.addWidget(self.image_label)
        top_layout.addWidget(self.robot_table)
        top_layout.addWidget(self.pose_label)

        bottom_layout = QVBoxLayout()
        bottom_layout.addWidget(self.reset_button)

        # 📍 목표 입력칸
        self.goal_input_x = QLineEdit()
        self.goal_input_y = QLineEdit()
        self.goal_input_yaw = QLineEdit()
        self.goal_input_x.setPlaceholderText("목표 x (m)")
        self.goal_input_y.setPlaceholderText("목표 y (m)")
        self.goal_input_yaw.setPlaceholderText("yaw (-180~180°)")

        # 📤 목표 발행 버튼
        self.goal_button = QPushButton("📤 목표 발행")
        self.goal_button.clicked.connect(self.publish_target_pose)

        # 수평 배치
        goal_input_layout = QHBoxLayout()
        goal_input_layout.addWidget(self.goal_input_x)
        goal_input_layout.addWidget(self.goal_input_y)
        goal_input_layout.addWidget(self.goal_input_yaw)
        goal_input_layout.addWidget(self.goal_button)

        # 배치 추가
        bottom_layout.addLayout(goal_input_layout)

        # 퍼블리셔 캐시 {robot_id: TargetPosePublisher}
        self.target_publishers = {}

        self.populate_robot_table()

        main_layout.addLayout(top_layout)
        main_layout.addLayout(bottom_layout)
        self.setLayout(main_layout)

    def set_webcam_thread(self, thread):
        self.webcam_thread = thread

    def reset_grid_corners(self):
        if self.webcam_thread:
            self.webcam_thread.p0 = None
            self.webcam_thread.p1 = None
            self.webcam_thread.p2 = None
            self.webcam_thread.p3 = None
            self.webcam_thread.grid_corners = None
        self.draw_grid_once = True
        if self.current_frame is not None:
            self.update_image(self.current_frame)

    def populate_robot_table(self):
        self.robot_table.setRowCount(len(self.robot_tag_map))
        for row, (rid, tid) in enumerate(self.robot_tag_map.items()):
            self.robot_table.setItem(row, 0, QTableWidgetItem(str(rid)))
            self.robot_table.setItem(row, 1, QTableWidgetItem(str(tid)))

    def on_table_click(self, row, column):
        self.selected_row = row
        self.update_selected_pose()  # 즉시 표시

    def update_selected_pose(self):
        if self.selected_row is None:
            return
        robot_id = int(self.robot_table.item(self.selected_row, 0).text())
        tag_id = self.robot_tag_map.get(robot_id)
        pose = _get_transed_pose_for_tag(self.webcam_thread, tag_id)
        if pose is None:
            self.pose_label.setText(
                f"🛰️ 위치 정보: (로봇 {robot_id}, 태그 {tag_id}) ➤ 감지되지 않음"
            )
        else:
            x, y, yaw = pose
            deg = np.degrees(yaw)
            self.pose_label.setText(
                f"🛰️ 위치 정보: 로봇 {robot_id}, 태그 {tag_id} ➤ x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({deg:.1f}°)"
            )

    def update_image(self, frame):
        self.current_frame = frame.copy()
        if self.webcam_thread:
            tag_poses = self.webcam_thread.get_all_tag_poses()
            for entry in tag_poses:
                corners = entry.get("corners")
                if corners:
                    for i in range(4):
                        cv2.line(
                            self.current_frame,
                            corners[i],
                            corners[(i + 1) % 4],
                            (0, 255, 0),
                            2,
                        )
                    cX, cY, _ = entry["pose"]
                    cv2.circle(self.current_frame, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.putText(
                        self.current_frame,
                        f"ID:{entry['id']}",
                        (cX + 5, cY - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 0),
                        3,
                    )
                    cv2.putText(
                        self.current_frame,
                        f"ID:{entry['id']}",
                        (cX + 5, cY - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )
            if self.draw_grid_once:
                self.webcam_thread.draw_grid_corners(self.current_frame)

        h, w, ch = self.current_frame.shape
        bytes_per_line = ch * w
        q_image = QImage(
            self.current_frame.data, w, h, bytes_per_line, QImage.Format.Format_BGR888
        )
        pixmap = QPixmap.fromImage(q_image)
        self.image_label.setPixmap(pixmap)

    def publish_target_pose(self):
        if self.selected_row is None:
            print("❌ 로봇을 선택하세요.")
            return

        try:
            x = float(self.goal_input_x.text())
            y = float(self.goal_input_y.text())
            yaw_deg = float(self.goal_input_yaw.text())
        except ValueError:
            print("❌ x, y, yaw 값을 모두 숫자로 입력하세요.")
            return

        yaw = np.radians(yaw_deg)

        robot_id = int(self.robot_table.item(self.selected_row, 0).text())
        tag_id = self.robot_tag_map.get(robot_id)

        # 퍼블리셔가 없으면 생성
        if robot_id not in self.target_publishers:
            self.target_publishers[robot_id] = TargetPosePublisher(robot_id, tag_id)

        self.target_publishers[robot_id].publish_target_pose(x, y, yaw)
        print(
            f"✅ 목표 위치 발행됨 → /robot{robot_id}/target_pose : x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    # 기존 WebcamThread의 visualize_grid 함수 복사
    def visualize_grid(self, img, grid_points, rows, cols):
        for row in range(rows):
            pts = [pt for r, c, pt in grid_points if r == row]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (255, 255, 0), 2)
        for col in range(cols):
            pts = [pt for r, c, pt in grid_points if c == col]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (0, 255, 255), 2)
        for row, col, pt in grid_points:
            cv2.circle(img, pt, 2, (0, 0, 255), 2)


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
        return lambda: _get_transed_pose_for_tag(webcam_thread, tag_id)

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


# ✅ 헬퍼 함수: tag_id에 해당하는 pose를 가져와 변환
def _get_transed_pose_for_tag(webcam_thread, tag_id):
    poses = webcam_thread.get_tag_pose(tag_id)
    if not poses:
        return None
    # 여러 개일 수 있으므로 첫 번째만 사용
    x, y, yaw = poses[0]

    if None in (webcam_thread.p0, webcam_thread.p1, webcam_thread.p2, webcam_thread.p3):
        return None

    # 변환
    grid_corners = np.array(
        [webcam_thread.p0, webcam_thread.p1, webcam_thread.p2, webcam_thread.p3],
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
    Minv = cv2.getPerspectiveTransform(grid_corners, unit_square)
    img_point = np.array([[[x, y]]], dtype=np.float32)
    relative_point = cv2.perspectiveTransform(img_point, Minv)
    u, v = relative_point[0][0]
    real_x = u * REAL_MAX_WIDTH
    real_y = REAL_MAX_HEIGHT - v * REAL_MAX_HEIGHT
    return real_x, real_y, yaw


if __name__ == "__main__":
    main()
