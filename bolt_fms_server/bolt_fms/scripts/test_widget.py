import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QTableWidget, QLabel, QSplitter, QGroupBox, QTableWidgetItem
)
from PySide6.QtCore import Qt

class YourMainWindow(QWidget):
    def __init__(self):
        super().__init__()

        # --- 테스트를 위한 임시 변수 및 객체 (실제 코드에서는 삭제) ---
        self.visualizer_widget = QLabel("--- Stock 위젯 (시각화 영역) ---")
        # Qt.AlignCenter -> Qt.AlignmentFlag.AlignCenter 로 변경
        self.visualizer_widget.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.visualizer_widget.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc;")
        
        # 실제 Manager 객체가 있다고 가정
        class MockManager:
            def __init__(self):
                self.robots = {'R1': None, 'R2': None, 'R3': None}
        self.manager = MockManager()
        # --- 테스트 코드 끝 ---

        self.init_ui()
        self.setWindowTitle("작업 관리 UI (PySide6)")
        self.resize(1200, 800)

    def init_ui(self):
        """작업 관리 UI 초기화"""
        main_layout = QVBoxLayout(self)

        # --- 메인 콘텐츠 영역: 시각화 위젯과 제어 패널을 좌우로 분리 ---
        # Qt.Horizontal -> Qt.Orientation.Horizontal 로 변경
        content_splitter = QSplitter(Qt.Orientation.Horizontal)

        # 1. 시각화 위젯 (왼쪽 패널)
        content_splitter.addWidget(self.visualizer_widget)

        # 2. 제어 패널 (오른쪽 패널)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # 2-1. 로봇 상태 테이블
        robot_status_group = QGroupBox("🤖 로봇 상태")
        robot_status_layout = QVBoxLayout(robot_status_group)
        
        self.robot_status_table = QTableWidget()
        self.robot_status_table.setColumnCount(6)
        robot_headers = ["로봇 ID", "타입", "상태", "현재 작업", "현재 위치", "배터리"]
        self.robot_status_table.setHorizontalHeaderLabels(robot_headers)
        self.robot_status_table.setMaximumHeight(200)
        robot_status_layout.addWidget(self.robot_status_table)
        
        right_layout.addWidget(robot_status_group)

        # 2-2. 버튼 컨트롤 영역
        control_group = QGroupBox("🕹️ 작업 제어")
        control_group_layout = QHBoxLayout(control_group)

        task_control_layout = QVBoxLayout()
        self.btn_add_inbound = QPushButton("📦 입고 작업 추가")
        self.btn_add_inbound.setStyleSheet("padding: 10px; font-size: 14px;")
        # self.btn_add_inbound.clicked.connect(self.test_add_inbound)
        task_control_layout.addWidget(self.btn_add_inbound)

        self.btn_add_outbound = QPushButton("📤 출고 작업 추가")
        self.btn_add_outbound.setStyleSheet("padding: 10px; font-size: 14px;")
        # self.btn_add_outbound.clicked.connect(self.test_add_outbound)
        task_control_layout.addWidget(self.btn_add_outbound)

        self.btn_assign = QPushButton("🚚 작업 할당 실행")
        self.btn_assign.setStyleSheet("padding: 10px; font-size: 14px; background-color: #4CAF50; color: white;")
        # self.btn_assign.clicked.connect(self.test_assign_tasks_waypoints)
        task_control_layout.addWidget(self.btn_assign)
        task_control_layout.addStretch(1)

        robot_complete_layout = QVBoxLayout()
        robot_complete_label = QLabel("로봇 작업 완료:")
        robot_complete_layout.addWidget(robot_complete_label)

        for robot_id in self.manager.robots.keys():
            btn = QPushButton(f"✅ 로봇 {robot_id} 완료")
            # btn.clicked.connect(lambda checked, rid=robot_id: self.complete_task(rid))
            btn.setStyleSheet("padding: 8px;")
            robot_complete_layout.addWidget(btn)
        robot_complete_layout.addStretch(1)

        control_group_layout.addLayout(task_control_layout)
        control_group_layout.addLayout(robot_complete_layout)
        right_layout.addWidget(control_group)
        right_layout.addStretch(1)

        content_splitter.addWidget(right_panel)
        content_splitter.setSizes([800, 400])
        main_layout.addWidget(content_splitter)

        # --- 하단 영역: 전체 작업 현황 테이블 ---
        task_table_group = QGroupBox("📊 전체 작업 현황")
        task_table_layout = QVBoxLayout(task_table_group)

        self.table = QTableWidget()
        self.table.setColumnCount(7)
        headers = ["Process ID", "Task ID", "작업 타입", "로봇 타입", "할당 로봇", "작업 상태", "목표 위치"]
        self.table.setHorizontalHeaderLabels(headers)
        task_table_layout.addWidget(self.table)
        
        main_layout.addWidget(task_table_group)
        main_layout.setStretchFactor(content_splitter, 3)
        main_layout.setStretchFactor(task_table_group, 1)

# --- 실행 코드 (테스트용) ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = YourMainWindow()
    window.show()
    # app.exec_() -> app.exec() 로 변경
    sys.exit(app.exec())