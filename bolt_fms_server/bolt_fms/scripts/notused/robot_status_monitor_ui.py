
import sys
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QTableWidget, QTableWidgetItem, QHeaderView, QProgressBar
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont
import json

class RobotStatusMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🤖 로봇 상태 모니터")
        self.setGeometry(100, 100, 1200, 800)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.setup_ui()

    def setup_ui(self):
        # Title
        title_label = QLabel("실시간 로봇 상태 대시보드")
        title_label.setFont(QFont("Arial", 24, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(title_label)

        # Table for robot statuses
        self.status_table = QTableWidget()
        self.status_table.setColumnCount(8)
        self.status_table.setHorizontalHeaderLabels([
            "로봇 ID", "상태", "작업", "배터리", "현재 위치",
            "목표 위치", "목표까지의 거리", "현재 속도"
        ])
        self.status_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.status_table.verticalHeader().setVisible(False)
        self.status_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.layout.addWidget(self.status_table)

    def update_status(self, statuses):
        robot_ids = sorted(statuses.keys(), key=int)
        self.status_table.setRowCount(len(robot_ids))

        for i, robot_id in enumerate(robot_ids):
            status = statuses[robot_id]
            
            # 로봇 ID
            self.status_table.setItem(i, 0, QTableWidgetItem(str(robot_id)))
            
            # 상태
            self.status_table.setItem(i, 1, QTableWidgetItem(status.get('state', 'N/A')))
            
            # 작업
            self.status_table.setItem(i, 2, QTableWidgetItem(status.get('task', 'N/A')))
            
            # 배터리
            battery_level = status.get('battery')
            if battery_level is not None:
                progress_bar = QProgressBar()
                progress_bar.setValue(int(battery_level * 100))
                progress_bar.setFormat(f"{battery_level*100:.1f}%")
                if battery_level < 0.2:
                    progress_bar.setStyleSheet("QProgressBar::chunk { background-color: red; }")
                elif battery_level < 0.5:
                    progress_bar.setStyleSheet("QProgressBar::chunk { background-color: orange; }")
                else:
                    progress_bar.setStyleSheet("QProgressBar::chunk { background-color: green; }")
                self.status_table.setCellWidget(i, 3, progress_bar)
            else:
                self.status_table.setItem(i, 3, QTableWidgetItem("N/A"))

            # 현재 위치
            pos = status.get('current_position', {})
            pos_str = f"({pos.get('x', 0):.2f}, {pos.get('y', 0):.2f})"
            self.status_table.setItem(i, 4, QTableWidgetItem(pos_str))

            # 목표 위치
            target_pos = status.get('target_position', {})
            target_pos_str = f"({target_pos.get('x', 0):.2f}, {target_pos.get('y', 0):.2f})"
            self.status_table.setItem(i, 5, QTableWidgetItem(target_pos_str))

            # 목표까지의 거리
            distance = status.get('distance_to_target')
            dist_str = f"{distance:.2f}m" if distance is not None else "N/A"
            self.status_table.setItem(i, 6, QTableWidgetItem(dist_str))

            # 현재 속도
            speed = status.get('current_speed')
            speed_str = f"{speed:.2f}m/s" if speed is not None else "N/A"
            self.status_table.setItem(i, 7, QTableWidgetItem(speed_str))

            # Align all items to center
            for j in range(self.status_table.columnCount()):
                item = self.status_table.item(i, j)
                if item:
                    item.setTextAlignment(Qt.AlignCenter)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    monitor = RobotStatusMonitor()
    
    # Example data for testing
    dummy_data = {
        "1": {
            "robot_id": 1, "state": "MOVING", "task": "Go to A", "battery": 0.85,
            "current_position": {"x": 1.2, "y": 3.4}, "target_position": {"x": 5.0, "y": 2.1},
            "distance_to_target": 4.1, "current_speed": 0.5
        },
        "2": {
            "robot_id": 2, "state": "IDLE", "task": "None", "battery": 0.45,
            "current_position": {"x": 7.8, "y": 1.1}, "target_position": {"x": 7.8, "y": 1.1},
            "distance_to_target": 0.0, "current_speed": 0.0
        },
        "3": {
            "robot_id": 3, "state": "CHARGING", "task": "None", "battery": 0.15,
            "current_position": {"x": 0.5, "y": 9.2}, "target_position": {"x": 0.5, "y": 9.2},
            "distance_to_target": 0.0, "current_speed": 0.0
        }
    }
    monitor.update_status(dummy_data)
    
    monitor.show()
    sys.exit(app.exec())
