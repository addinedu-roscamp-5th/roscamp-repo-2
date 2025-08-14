import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QTabWidget, QHeaderView
from PyQt5.QtCore import Qt

from models import Base, Rack, Location, Inventory, Item, Orders, Orders_Item, Outbound, Inbound
from database import get_db

from sqlalchemy.orm import Session
from sqlalchemy import create_engine

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("창고 관리 시스템")
        self.setGeometry(100, 100, 1200, 800)

        self.db_session = next(get_db())

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.tab_widget = QTabWidget()
        self.layout.addWidget(self.tab_widget)

        self.setup_tabs()

    def setup_tabs(self):
        # 탭 생성
        inbound_tab = QWidget()
        outbound_tab = QWidget()
        inventory_tab = QWidget()

        self.tab_widget.addTab(inbound_tab, "입고 내역")
        self.tab_widget.addTab(outbound_tab, "출고 내역")
        self.tab_widget.addTab(inventory_tab, "재고 내역")

        # 각 탭에 테이블 위젯 추가
        self.inbound_table = self.create_table_widget(inbound_tab, "입고 내역")
        self.outbound_table = self.create_table_widget(outbound_tab, "출고 내역")
        self.inventory_table = self.create_table_widget(inventory_tab, "재고 내역")

        # 데이터 로드
        self.load_inbound_data()
        self.load_outbound_data()
        self.load_inventory_data()

    def create_table_widget(self, parent_widget, tab_name):
        layout = QVBoxLayout(parent_widget)
        table = QTableWidget()
        layout.addWidget(table)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        return table
    
    def load_inbound_data(self):
        # 헤더 수정: item_amount -> '박스당 개수', ib_amount -> '입고 박스 수량', '총 입고 수량' 추가
        headers = ["입고 ID", "상품 ID", "상품명", "박스당 개수", "입고 박스 수량", "총 입고 수량", "상태", "입고 일시"]
        self.inbound_table.setColumnCount(len(headers))
        self.inbound_table.setHorizontalHeaderLabels(headers)
        
        # Inbound와 Item 테이블 조인 후, 입고 ID(ib_id) 기준으로 오름차순 정렬
        results = self.db_session.query(
            Inbound.ib_id,
            Inbound.item_id,
            Item.item_name,
            Inbound.item_amount,
            Inbound.ib_amount,
            Inbound.ib_status,
            Inbound.ib_dttm
        ).join(Item, Inbound.item_id == Item.item_id).order_by(Inbound.ib_id.asc()).all()

        self.inbound_table.setRowCount(len(results))
        for row_idx, row_data in enumerate(results):
            # 총 입고 수량 계산
            total_amount = row_data[3] * row_data[4]
            
            # 테이블 위젯에 데이터 채우기
            for col_idx, data in enumerate(row_data):
                item = QTableWidgetItem(str(data))
                if col_idx == 4: # '입고 박스 수량' 바로 옆에 '총 입고 수량'을 추가
                    self.inbound_table.setItem(row_idx, col_idx, item)
                    self.inbound_table.setItem(row_idx, col_idx + 1, QTableWidgetItem(str(total_amount)))
                else:
                    # 총 입고 수량 열을 고려하여 컬럼 인덱스 조정
                    adj_col_idx = col_idx
                    if col_idx > 4:
                        adj_col_idx += 1
                    self.inbound_table.setItem(row_idx, adj_col_idx, item)

    def load_outbound_data(self):
        # 헤더 설정 (요청에 따라 컬럼 순서 및 '상품명' 추가)
        headers = ["주문 ID", "상품 ID", "상품명", "주문 수량", "주문 상태", "배송지", "주문 일시"]
        self.outbound_table.setColumnCount(len(headers))
        self.outbound_table.setHorizontalHeaderLabels(headers)
        
        # Orders, Orders_Item, Item 테이블을 조인하여 데이터 조회
        results = self.db_session.query(
            Orders.order_id,
            Orders_Item.item_id,
            Item.item_name,
            Orders_Item.order_amount,
            Orders.order_status,
            Orders.destination,
            Orders.order_dttm
        ).join(Orders_Item, Orders.order_id == Orders_Item.order_id) \
        .join(Item, Orders_Item.item_id == Item.item_id).all()
        
        self.outbound_table.setRowCount(len(results))
        for row_idx, row_data in enumerate(results):
            for col_idx, data in enumerate(row_data):
                item = QTableWidgetItem(str(data))
                self.outbound_table.setItem(row_idx, col_idx, item)

    def load_inventory_data(self):
        # 헤더 설정
        headers = ["위치 ID", "상품 ID", "상품명", "입고 일시"]
        self.inventory_table.setColumnCount(len(headers))
        self.inventory_table.setHorizontalHeaderLabels(headers)
        
        # Inventory와 Item 테이블 조인하여 데이터 조회
        results = self.db_session.query(
            Inventory.location_id,
            Inventory.item_id,
            Item.item_name,
            Inventory.iv_dttm
        ).join(Item, Inventory.item_id == Item.item_id).order_by(Inventory.location_id.asc()).all()

        # 테이블 위젯에 데이터 채우기
        self.inventory_table.setRowCount(len(results))
        for row_idx, row_data in enumerate(results):
            for col_idx, data in enumerate(row_data):
                item = QTableWidgetItem(str(data))
                self.inventory_table.setItem(row_idx, col_idx, item)

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()