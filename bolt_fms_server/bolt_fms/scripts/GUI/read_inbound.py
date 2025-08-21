#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import requests
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout,
    QHBoxLayout, QFormLayout, QGroupBox, QTableWidget, QTableWidgetItem, QMessageBox
)
from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont

FASTAPI_SERVER_URL = "http://127.0.0.1:8000"

# ─────────────────────────────────────
# PySide6 GUI 애플리케이션
# ─────────────────────────────────────
class InboundApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.load_inbound_data()

    def initUI(self):
        self.setWindowTitle('물류센터 입고 관리 시스템 (FastAPI + ROS2)')
        self.setGeometry(100, 100, 1200, 800)
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # ───────── 입고 테이블 ─────────
        self.table = QTableWidget(self)
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(['입고 ID', '상품 ID', '상품 개수', '박스 수량', '입고일시', '상태'])
        self.table.setColumnWidth(4, 200)
        main_layout.addWidget(self.table)

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


# ─────────────────────────────────────
# 앱 실행 진입점
# ─────────────────────────────────────
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = InboundApp()
    ex.show()
    sys.exit(app.exec())
