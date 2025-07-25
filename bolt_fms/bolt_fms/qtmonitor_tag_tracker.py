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

from PySide6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton
from PySide6.QtGui import QImage, QPixmap, QMouseEvent

import sys
print("---------------------------------------------------------------")
print(sys.executable)  # 실행 중인 파이썬 경로 확인



from pupil_apriltags import Detector

# === 설정 ===
TAG_ID = 2
CAMERA_INDEX = 3
VISUALIZE = True

# === 그리드 설정 ===
CAP_RATIO = 0.75

CAP_WIDTH = 1920 * CAP_RATIO
CAP_HEIGHT = 1080 * CAP_RATIO

RESIZE_RATIO = 1

DISPLAY_WIDTH = CAP_WIDTH * RESIZE_RATIO
DISPLAY_HEIGHT = CAP_HEIGHT * RESIZE_RATIO

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
        super().__init__('multi_pose_publisher')
        self.pose_publishers = {}  # {id: publisher}
        self.get_pose_func_map = get_pose_func_map  # {id: func}

        for robot_id in robot_ids:
            topic = f'/robot{robot_id}/camera_pose'
            self.pose_publishers[robot_id] = self.create_publisher(PoseStamped, topic, 10)
            self.create_timer(publish_period, lambda rid=robot_id: self.publish_pose(rid))

    def publish_pose(self, robot_id):
        pose_func = self.get_pose_func_map[robot_id]
        pose = pose_func()
        if pose is None:
            return

        x, y, yaw = pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.pose_publishers[robot_id].publish(msg)
        self.get_logger().info(f"📤 /robot{robot_id}/camera_pose → x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

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
        self.tag_center_set = set()
        self.all_tag_poses = []

    # 보간 함수
    def interpolate(self, p1, p2, t):
        return (int(p1[0] + t * (p2[0] - p1[0])), int(p1[1] + t * (p2[1] - p1[1])))

    def generate_grid_points(self, pts, h_div, v_div):
        self.p0, self.p1, self.p2, self.p3 = pts

        lefts = [self.interpolate(self.p0, self.p2, i / h_div) for i in range(h_div + 1)]
        rights = [self.interpolate(self.p1, self.p3, i / h_div) for i in range(h_div + 1)]

        tops = [self.interpolate(self.p0, self.p1, i / v_div) for i in range(v_div + 1)]
        bottoms = [self.interpolate(self.p2, self.p3, i / v_div) for i in range(v_div + 1)]

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
    def find_closest(self, center, candidates):
        min_dist = float('inf')
        closest = center
        for c in candidates:
            dist = np.linalg.norm(np.array(c) - np.array(center))
            if dist < min_dist:
                min_dist = dist
                closest = c
        return tuple(closest)
    
    def get_tag_pose(self, tag_id):
        with self.result_lock:
            return [entry["pose"] for entry in self.all_tag_poses if entry["id"] == tag_id]

    def get_all_tag_poses(self):
        with self.result_lock:
            return list(self.all_tag_poses)  # 사본 반환

    def draw_grid_corners(self, frame, corners):
        if corners:
            grid_points = self.generate_grid_points(corners, horizontal_divisions, vertical_divisions)
            self.visualize_grid(frame, grid_points, ROWS, COLS)


    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)

        if not cap.isOpened():
            print("❌ 웹캠 열기 실패")
            return

        detector = Detector(families='tag36h11')
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
                current_tag_poses_list.append({
                    "id": tag.tag_id,
                    "pose": (cX, cY, -yaw),
                    "corners": [(int(p[0]), int(p[1])) for p in tag.corners]
                })
                if tag.tag_id == self.tag_id:
                    with self.result_lock:
                        self.pose = (cX, cY, -yaw)

            with self.result_lock:
                self.all_tag_poses = current_tag_poses_list

            # 프레임만 Qt로 전달 (resize 제거!)
            if self.frame_callback:
                self.frame_callback(frame)  # 원본 크기 그대로 전달

        cap.release()
        cv2.destroyAllWindows()
        print("🛑 웹캠 스레드 종료")

    def stop(self):
        self.running = False


class ImageWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AprilTag Viewer (Qt)")
        self.image_label = QLabel()
        self.image_label.setFixedSize(int(DISPLAY_WIDTH), int(DISPLAY_HEIGHT))  # 크기 고정
        self.image_label.mousePressEvent = self.mouse_click_event

        self.points = []
        self.manual_mode = False
        self.current_frame = None
        self.webcam_thread = None

        self.button = QPushButton("🖱️ p0~p3 수동 선택 모드")
        self.button1 = QPushButton("Reset")
        self.button.clicked.connect(self.enable_manual_mode)
        self.button1.clicked.connect(self.reset)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.button)
        layout.addWidget(self.button1)
        self.setLayout(layout)

    def reset(self):
        self.webcam_thread.printed_once = True
        self.webcam_thread.grid_corners = []

    def enable_manual_mode(self):
        self.points = []
        self.manual_mode = True

    def set_webcam_thread(self, thread):
        self.webcam_thread = thread

    def update_image(self, frame):
        self.current_frame = frame.copy()
        draw_frame = frame.copy()

        # 태그, 그리드 등 시각화
        if self.webcam_thread:
            tag_poses = self.webcam_thread.get_all_tag_poses()
            # 태그 사각형 및 ID 표시
            for entry in tag_poses:
                corners = entry.get("corners")
                if corners:
                    for i in range(4):
                        cv2.line(draw_frame, corners[i], corners[(i+1)%4], (0,255,0), 2)
                    cX, cY, _ = entry["pose"]
                    cv2.circle(draw_frame, (cX, cY), 5, (0,0,255), -1)
                    cv2.putText(draw_frame, f"ID:{entry['id']}", (cX+5, cY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 4)
                    cv2.putText(draw_frame, f"ID:{entry['id']}", (cX+5, cY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            # 그리드 그리기
            corners = self.webcam_thread.grid_corners
            if corners and len(corners) == 4:
                grid_points = self.webcam_thread.generate_grid_points(corners, horizontal_divisions, vertical_divisions)
                self.visualize_grid(draw_frame, grid_points, ROWS, COLS)

        # 수동 선택 점 표시
        if self.points:
            for i, pt in enumerate(self.points):
                cv2.circle(draw_frame, pt, 5, (0, 0, 255), -1)
                cv2.putText(draw_frame, f"p{i}", (pt[0]+5, pt[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            if len(self.points) == 4:
                cv2.polylines(draw_frame, [np.array(self.points, np.int32)], isClosed=True, color=(255, 0, 0), thickness=2)

        h, w, ch = draw_frame.shape
        bytes_per_line = ch * w
        q_image = QImage(draw_frame.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)
        pixmap = QPixmap.fromImage(q_image)
        self.image_label.setPixmap(pixmap)

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

    def mouse_click_event(self, event: QMouseEvent):
        if not self.manual_mode or self.current_frame is None:
            return

        x = int(event.position().x())
        y = int(event.position().y())

        if len(self.points) < 4:
            self.points.append((x, y))

        if len(self.points) == 4:
            self.manual_mode = False
            if self.webcam_thread:
                self.webcam_thread.p0 = self.points[0]
                self.webcam_thread.p1 = self.points[1]
                self.webcam_thread.p2 = self.points[2]
                self.webcam_thread.p3 = self.points[3]
                self.webcam_thread.grid_corners = [self.webcam_thread.p0, self.webcam_thread.p1, self.webcam_thread.p2, self.webcam_thread.p3]

        self.update_image(self.current_frame)


def main():

    app = QApplication(sys.argv)
    image_window = ImageWindow()
    image_window.show()

    # 스레드 실행 시 콜백 전달
    webcam_thread = WebcamThread(tag_id=TAG_ID, camera_index=CAMERA_INDEX, visualize=True, frame_callback=image_window.update_image)
    webcam_thread.start()

    image_window.set_webcam_thread(webcam_thread)
    
    rclpy.init(args=None)

    robot_ids = [2,4]

    # pose 함수 매핑 생성
    def make_pose_func(tag_id):
        return lambda: _get_transed_pose_for_tag(webcam_thread, tag_id)

    get_pose_func_map = {rid: make_pose_func(rid) for rid in robot_ids}

    # 노드 실행
    multi_publisher_node = PoseMultiPublisher(robot_ids, get_pose_func_map, publish_period=1)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(multi_publisher_node), daemon=True)
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
    grid_corners = np.array([webcam_thread.p0, webcam_thread.p1, webcam_thread.p2, webcam_thread.p3], dtype=np.float32)
    unit_square = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
        [1.0, 1.0],
    ], dtype=np.float32)
    Minv = cv2.getPerspectiveTransform(grid_corners, unit_square)
    img_point = np.array([[[x, y]]], dtype=np.float32)
    relative_point = cv2.perspectiveTransform(img_point, Minv)
    u, v = relative_point[0][0]
    real_x = u * REAL_MAX_WIDTH
    real_y = REAL_MAX_HEIGHT - v * REAL_MAX_HEIGHT
    return real_x, real_y, yaw

if __name__ == '__main__':
    main()
