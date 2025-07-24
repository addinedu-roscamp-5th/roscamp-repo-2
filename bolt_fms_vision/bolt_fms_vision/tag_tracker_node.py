import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import atan2, sin, cos
from pupil_apriltags import Detector
import cv2
import numpy as np

from PySide6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer
import sys
import cv2
import numpy as np


import sys
print("---------------------------------------------------------------")
print(sys.executable)  # 실행 중인 파이썬 경로 확인



from pupil_apriltags import Detector

# === 설정 ===
TAG_ID = 2
CAMERA_INDEX = 2
VISUALIZE = True

# === 그리드 설정 ===
CAP_WIDTH = 1280
CAP_HEIGHT = 720

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
class PeriodicInitialPosePublisher(Node):
    def __init__(self, get_pose_func, publish_period=5.0):
        super().__init__('camera_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, '/camera_pose', 10)
        self.get_pose_func = get_pose_func
        self.timer = self.create_timer(publish_period, self.publish_pose)

    def publish_pose(self):
        pose = self.get_pose_func()
        if pose is None:
            return

        x, y, yaw = pose
        now = self.get_clock().now()

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.pub.publish(msg)
        self.get_logger().info(f"🛰️ /camera_pose → x:{x:.3f}, y:{y:.3f}, yaw:{yaw:.3f} rad")

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
        self.printed_once = True  # 태그 ID 한 번만 출력
        self.grid_corners = None   # trapezoid 꼭짓점 저장
        self.tag_center_set = set() 

    def get_pose(self):
        with self.result_lock:
            return self.pose
    
    def get_transed_pose(self):
        with self.result_lock:
            if self.pose is None:
                return None
            x, y, yaw = self.pose
            if None in (self.p0, self.p1, self.p2, self.p3):
                return None

            # 원근 변환 역행렬 계산 (이미지 → 상대좌표)
            grid_corners = np.array([self.p0, self.p1, self.p2, self.p3], dtype=np.float32)
            unit_square = np.array([
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
            ], dtype=np.float32)

            Minv = cv2.getPerspectiveTransform(grid_corners, unit_square)

            # 현재 (x, y) → (u, v) 상대 좌표
            img_point = np.array([[[x, y]]], dtype=np.float32)
            relative_point = cv2.perspectiveTransform(img_point, Minv)
            u, v = relative_point[0][0]  # 상대 좌표계 [0.0 ~ 1.0]

            # 실제 물리 좌표계로 변환
            real_x = u * REAL_MAX_WIDTH
            real_y = REAL_MAX_HEIGHT - v * REAL_MAX_HEIGHT

            return real_x, real_y, yaw

        
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
    

    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)

        if not cap.isOpened():
            print("❌ 웹캠 열기 실패")
            return

        detector = Detector(families='tag36h11')
        print(f"🎯 AprilTag ID {self.tag_id} 추적 중...")

        # tag_centers = []              # [[x, y], ...]
        # tag_id_to_center = {}         # {id: (x, y)}
        center_to_tag_id = {}         # {(x, y): id}


        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("오류: 프레임을 읽을 수 없습니다.")
                break

            h, w = frame.shape[:2]

            top_left     = (0, 0)
            top_right    = (w - 1, 0)
            bottom_left  = (0, h - 1)
            bottom_right = (w - 1, h - 1)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray, estimate_tag_pose=False)

            for tag in tags:
                # if tag.tag_id == self.tag_id:
                cX, cY = int(tag.center[0]), int(tag.center[1])
                pt0, pt1 = tag.corners[0], tag.corners[1]
                # tag_centers.append([cX, cY])
                # tag_id_to_center[tag.tag_id] = (cX, cY)
                center_to_tag_id[(cX, cY)] = tag.tag_id
                self.tag_center_set.add((cX, cY))

                dx = pt1[0] - pt0[0]
                dy = pt1[1] - pt0[1]
                yaw = atan2(dy, dx)
                if tag.tag_id == self.tag_id:
                    with self.result_lock:
                        self.pose = (cX, cY, -yaw)

                if self.visualize:
                    corners = [(int(p[0]), int(p[1])) for p in tag.corners]
                    for i in range(4):
                        cv2.line(frame, corners[i], corners[(i+1)%4], (0,255,0), 2)
                    cv2.circle(frame, (cX, cY), 5, (0,0,255), -1)
                    cv2.putText(frame, f"ID:{tag.tag_id}", (cX+5, cY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 4)
                    # 원래 글자(흰색) 다시 그림
                    cv2.putText(frame, f"ID:{tag.tag_id}", (cX+5, cY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            if len(self.tag_center_set) >= 4 and self.printed_once:
                tag_center_list = list(self.tag_center_set)  # [(x,y), ...]

                self.grid_corners = [
                    self.find_closest(top_left, tag_center_list),
                    self.find_closest(top_right, tag_center_list),
                    self.find_closest(bottom_left, tag_center_list),
                    self.find_closest(bottom_right, tag_center_list)
                ]
                # self.printed_once = False
                # print("📌 보정된 꼭짓점 위치(grid_corners):", self.grid_corners)


            if self.grid_corners:
                grid_points = self.generate_grid_points(self.grid_corners, horizontal_divisions, vertical_divisions)
                self.visualize_grid(frame, grid_points, ROWS, COLS)

            # # ✅ Qt를 위한 콜백 처리
            if self.frame_callback:
                self.frame_callback(frame.resize(CAP_WIDTH,CAP_HEIGHT))  # Qt로 이미지 전송

            # # 웹캠 프레임 시각화
            # if self.visualize:
            #     cv2.imshow("AprilTag Tracker", frame)
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         break


        cap.release()
        cv2.destroyAllWindows()
        print("🛑 웹캠 스레드 종료")

    def stop(self):
        self.running = False


from PySide6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton
from PySide6.QtGui import QImage, QPixmap, QMouseEvent
from PySide6.QtCore import Qt
import cv2
import numpy as np

class ImageWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AprilTag Viewer (Qt)")
        self.image_label = QLabel()
        self.image_label.setScaledContents(True)
        self.image_label.mousePressEvent = self.mouse_click_event

        self.points = []  # 수동 선택된 4개 점
        self.manual_mode = False
        self.current_frame = None
        self.webcam_thread = None  # webcam_thread 연결 예정

        self.button = QPushButton("🖱️ p0~p3 수동 선택 모드")
        self.button.clicked.connect(self.enable_manual_mode)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.button)
        self.setLayout(layout)
        # self.resize(CAP_WIDTH, CAP_HEIGHT)

    def enable_manual_mode(self):
        self.points = []
        self.manual_mode = True

    def set_webcam_thread(self, thread):
        self.webcam_thread = thread

    def update_image(self, frame):
        self.current_frame = frame.copy()
        if self.points:
            for i, pt in enumerate(self.points):
                cv2.circle(frame, pt, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"p{i}", (pt[0]+5, pt[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            if len(self.points) == 4:
                cv2.polylines(frame, [np.array(self.points, np.int32)], isClosed=True, color=(255, 0, 0), thickness=2)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        q_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_BGR888)
        pixmap = QPixmap.fromImage(q_image)
        self.image_label.setPixmap(pixmap)

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
    rclpy.init()

    app = QApplication(sys.argv)
    image_window = ImageWindow()
    image_window.show()

    # 스레드 실행 시 콜백 전달
    webcam_thread = WebcamThread(tag_id=TAG_ID, camera_index=CAMERA_INDEX, visualize=True, frame_callback=image_window.update_image)
    webcam_thread.start()

    # ROS 노드 생성 대기
    while webcam_thread.get_pose() is None:
        time.sleep(0.1)
    # Qt 앱 시작

    image_window.set_webcam_thread(webcam_thread)
    ros_node = PeriodicInitialPosePublisher(get_pose_func=webcam_thread.get_transed_pose, publish_period=0.03)

    # ROS 노드 실행을 별도 쓰레드에서
    def ros_spin():
        rclpy.spin(ros_node)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    # ros_thread.start()

    # Qt 메인 루프 실행
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("🛑 종료 요청 받음")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        webcam_thread.stop()
        webcam_thread.join()
        print("✅ 프로그램 종료")



if __name__ == '__main__':
    main()
