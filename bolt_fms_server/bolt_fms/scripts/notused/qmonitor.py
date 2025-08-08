import sys
import threading
import time

from PySide6.QtWidgets import QApplication, QMainWindow, QListWidget, QVBoxLayout, QWidget, QLabel, QPushButton
from PySide6.QtCore import QTimer

from robot_status_subscriber import RobotStatusSubscriber

import rclpy

class RobotStatusQtWindow(QMainWindow):
    def __init__(self, robot_ids):
        super().__init__()
        self.setWindowTitle("로봇 상태 모니터링")
        self.resize(500, 400)
        self.robot_ids = robot_ids

        # UI 구성
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.status_label = QLabel("ROS2 상태: 초기화 중...")
        layout.addWidget(self.status_label)

        self.status_list = QListWidget()
        layout.addWidget(self.status_list)

        self.refresh_button = QPushButton("수동 새로고침")
        layout.addWidget(self.refresh_button)
        self.refresh_button.clicked.connect(self.update_status_view)

        # ROS2 초기화 및 구독자 실행
        self.ros_thread = None
        self.subscriber = None
        self._ros_ready = False
        self.start_ros2_subscriber()

        # 타이머: 주기적 UI 갱신
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status_view)
        self.timer.start(1000)  # 1초마다 갱신

    def start_ros2_subscriber(self):
        def ros_spin():
            try:
                rclpy.init(args=None)
                self.subscriber = RobotStatusSubscriber(self.robot_ids)
                self._ros_ready = True
                self.status_label.setText("ROS2 상태: 연결됨")
                rclpy.spin(self.subscriber)
            except Exception as e:
                self.status_label.setText(f"ROS2 오류: {e}")
        # 별도 쓰레드로 돌림 (메인스레드에서 Qt만)
        self.ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self.ros_thread.start()

    def update_status_view(self):
        if not self._ros_ready or not self.subscriber:
            return
        self.status_list.clear()
        for robot_id in self.robot_ids:
            status = self.subscriber.get_robot_status(robot_id)
            if not status:
                self.status_list.addItem(f"로봇{robot_id}: 상태 없음")
                continue
            # 각 필드별로 보기 좋게 출력
            cam_pose = status['camera_pose']
            
            pose_str = ""
            if cam_pose:
                x = cam_pose.pose.position.x
                y = cam_pose.pose.position.y
                pose_str = f"({x:.2f}, {y:.2f})"
            battery = status['battery'].data if status['battery'] else None
            task = status['task'].data if status['task'] else ""
            state = status['state'].data if status['state'] else ""
            robot_type = status['robot_type'].data if status['robot_type'] else ""
            self.status_list.addItem(
                f"로봇{robot_id} | 위치:{pose_str} | 배터리:{battery} | 작업:{task} | 상태:{state} | 타입:{robot_type}"
            )
    def closeEvent(self, event):
        print("종료: ROS2 shutdown")
        try:
            rclpy.shutdown()
        except:
            pass
        event.accept()

def main():
    app = QApplication(sys.argv)
    robot_ids = [1, 2, 3]
    window = RobotStatusQtWindow(robot_ids)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
