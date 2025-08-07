#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from bolt_fms.msg import InboundUpdate
import requests
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout,
    QHBoxLayout, QFormLayout, QGroupBox, QTableWidget, QTableWidgetItem, QMessageBox
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

FASTAPI_SERVER_URL = "http://192.168.0.139:8000"

# ─────────────────────────────────────
# ROS2 퍼블리셔 노드
# ─────────────────────────────────────
class WorkerNode(Node):
    def __init__(self):
        super().__init__('inbound_gui_pub_node')
        self.publisher_ = self.create_publisher(InboundUpdate, 'inbound_update', 10)

    def publish_inbound(self, item_id, item_amount, ib_amount):
        msg = InboundUpdate()
        msg.item_id = item_id
        msg.item_amount = item_amount
        msg.ib_amount = ib_amount
        self.publisher_.publish(msg)
        self.get_logger().info(f"📦 ROS2 발행: ID={item_id}, 수량={item_amount}, 박스={ib_amount}")

# ─────────────────────────────────────
# PyQt GUI 애플리케이션
# ─────────────────────────────────────
class InboundApp(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()
        self.load_inbound_data()

    def initUI(self):
        self.setWindowTitle('물류센터 입고 관리 시스템 (FastAPI + ROS2)')
        self.setGeometry(100, 100, 1200, 800)
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # 상단 레이아웃
        top_layout = QHBoxLayout()

        # ───────── 입고 정보 입력 ─────────
        inbound_group = QGroupBox("입고 정보 입력", self)
        inbound_layout = QFormLayout()

        self.item_id_input = QLineEdit()
        self.item_amount_input = QLineEdit()
        self.ib_amount_input = QLineEdit()

        inbound_layout.addRow('상품 ID:', self.item_id_input)
        inbound_layout.addRow('한 박스 당 개수:', self.item_amount_input)
        inbound_layout.addRow('입고 박스 수량:', self.ib_amount_input)

        add_button = QPushButton('입고 정보 저장')
        add_button.clicked.connect(self.add_inbound_item)
        inbound_layout.addRow(add_button)
        inbound_group.setLayout(inbound_layout)
        top_layout.addWidget(inbound_group)

        # ───────── 기능 버튼 ─────────
        function_group = QGroupBox("기능", self)
        function_layout = QVBoxLayout()

        self.ib_id_query_input = QLineEdit()
        query_button = QPushButton('입고 ID로 단일 조회')
        query_button.clicked.connect(self.query_inbound_by_id)

        update_status_button = QPushButton("선택된 행 상태 업데이트 (0 → 1)")
        update_status_button.clicked.connect(self.update_selected_inbound_status)

        refresh_button = QPushButton("테이블 새로고침")
        refresh_button.clicked.connect(self.load_inbound_data)

        function_layout.addWidget(QLabel("조회할 입고 ID:"))
        function_layout.addWidget(self.ib_id_query_input)
        function_layout.addWidget(query_button)
        function_layout.addWidget(update_status_button)
        function_layout.addWidget(refresh_button)

        function_group.setLayout(function_layout)
        top_layout.addWidget(function_group)

        main_layout.addLayout(top_layout)

        # ───────── 입고 테이블 ─────────
        self.table = QTableWidget(self)
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(['입고 ID', '상품 ID', '상품 개수', '박스 수량', '입고일시', '상태'])
        self.table.setColumnWidth(4, 200)
        main_layout.addWidget(self.table)

    # ─────────────────────────────
    # FastAPI + ROS2 연동: POST + Pub
    # ─────────────────────────────
    def add_inbound_item(self):
        try:
            item_id = int(self.item_id_input.text())
            item_amount = int(self.item_amount_input.text())
            ib_amount = int(self.ib_amount_input.text())

            if item_id <= 0 or item_amount <= 0 or ib_amount <= 0:
                raise ValueError("모든 값은 양수여야 합니다.")

            payload = {
                "item_id": item_id,
                "item_amount": item_amount,
                "ib_amount": ib_amount
            }

            response = requests.post(f"{FASTAPI_SERVER_URL}/inbound", json=payload)
            response.raise_for_status()
            result = response.json()

            if result.get("status") == "success":
                QMessageBox.information(self, '성공', f'입고 완료 (ID: {result["ib_id"]})')

                # ✅ ROS2 퍼블리시 추가
                self.ros_node.publish_inbound(item_id, item_amount, ib_amount)

                self.item_id_input.clear()
                self.item_amount_input.clear()
                self.ib_amount_input.clear()
                self.load_inbound_data()
            else:
                QMessageBox.warning(self, '서버 오류', '입고 정보 저장 실패')
        except Exception as e:
            QMessageBox.critical(self, '오류', f'입력 오류 또는 서버 오류: {e}')

    def load_inbound_data(self):
        self.table.setRowCount(0)
        try:
            response = requests.get(f"{FASTAPI_SERVER_URL}/inbounds")
            response.raise_for_status()
            items = response.json()
            for row_num, item in enumerate(items):
                self.table.insertRow(row_num)
                self.table.setItem(row_num, 0, QTableWidgetItem(str(item["ib_id"])))
                self.table.setItem(row_num, 1, QTableWidgetItem(str(item["item_id"])))
                self.table.setItem(row_num, 2, QTableWidgetItem(str(item["item_amount"])))
                self.table.setItem(row_num, 3, QTableWidgetItem(str(item["ib_amount"])))
                self.table.setItem(row_num, 4, QTableWidgetItem(str(item["ib_dttm"])))
                self.table.setItem(row_num, 5, QTableWidgetItem(str(item["ib_status"])))
        except Exception as e:
            QMessageBox.critical(self, '오류', f'데이터 로드 실패: {e}')

    def update_selected_inbound_status(self):
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            QMessageBox.warning(self, '선택 오류', '업데이트할 행을 선택하세요.')
            return

        try:
            ib_id = int(self.table.item(rows[0].row(), 0).text())
            response = requests.patch(f"{FASTAPI_SERVER_URL}/inbound/{ib_id}/status", json={"ib_status": 1})
            response.raise_for_status()
            result = response.json()
            if result.get("status") == "success":
                QMessageBox.information(self, '성공', f'입고 ID {ib_id} 상태가 1로 업데이트됨')
                self.load_inbound_data()
        except Exception as e:
            QMessageBox.critical(self, '업데이트 오류', f'상태 업데이트 실패: {e}')

    def query_inbound_by_id(self):
        try:
            ib_id = int(self.ib_id_query_input.text())
            response = requests.get(f"{FASTAPI_SERVER_URL}/inbound/{ib_id}")
            response.raise_for_status()
            data = response.json()
            QMessageBox.information(self, f'입고 ID {ib_id}', f"""
ID: {data['ib_id']}
상품 ID: {data['item_id']}
상품 개수: {data['item_amount']}
박스 수량: {data['ib_amount']}
상태: {data['ib_status']}
입고일시: {data['ib_dttm']}
            """)
        except Exception as e:
            QMessageBox.critical(self, '조회 오류', f'입고 조회 실패: {e}')

# ─────────────────────────────────────
# 앱 실행 진입점
# ─────────────────────────────────────
def main():
    rclpy.init()
    app = QApplication(sys.argv)

    ros_node = WorkerNode()
    gui = InboundApp(ros_node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
