import sys
import requests
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLineEdit, QPushButton, QTableWidget, QTableWidgetItem, QMessageBox, QLabel, QGroupBox
from PyQt5.QtCore import Qt

# FastAPI 서버의 기본 URL을 설정합니다.
FASTAPI_SERVER_URL = "http://127.0.0.1:8000"

class InboundApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.load_inbound_data()

    def initUI(self):
        """
        GUI 사용자 인터페이스를 초기화합니다.
        """
        self.setWindowTitle('물류센터 입고 관리 시스템 (FastAPI 연동)')
        self.setGeometry(100, 100, 1200, 800)

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # 상단 입력 필드 및 버튼 그룹
        top_layout = QHBoxLayout()

        # 입고 정보 입력 그룹
        inbound_group = QGroupBox("입고 정보 입력", self)
        inbound_layout = QFormLayout()
        
        self.item_id_input = QLineEdit(self)
        self.item_amount_input = QLineEdit(self)
        self.ib_amount_input = QLineEdit(self)

        inbound_layout.addRow('상품 ID:', self.item_id_input)
        inbound_layout.addRow('한 박스 당 개수:', self.item_amount_input)
        inbound_layout.addRow('입고 박스 수량:', self.ib_amount_input)

        add_button = QPushButton('입고 정보 저장', self)
        add_button.clicked.connect(self.add_inbound_item)
        inbound_layout.addRow(add_button)
        
        inbound_group.setLayout(inbound_layout)
        top_layout.addWidget(inbound_group)

        # 기능 버튼 그룹
        function_group = QGroupBox("기능", self)
        function_layout = QVBoxLayout()

        self.ib_id_query_input = QLineEdit(self)
        query_button = QPushButton('입고 ID로 단일 조회', self)
        query_button.clicked.connect(self.query_inbound_by_id)
        
        update_status_button = QPushButton("선택된 행 상태 업데이트 (0 -> 1)", self)
        update_status_button.clicked.connect(self.update_selected_inbound_status)
        
        refresh_button = QPushButton("테이블 새로고침", self)
        refresh_button.clicked.connect(self.load_inbound_data)
        
        function_layout.addWidget(QLabel("조회할 입고 ID:"))
        function_layout.addWidget(self.ib_id_query_input)
        function_layout.addWidget(query_button)
        function_layout.addWidget(update_status_button)
        function_layout.addWidget(refresh_button)
        
        function_group.setLayout(function_layout)
        top_layout.addWidget(function_group)
        
        main_layout.addLayout(top_layout)
        
        # 테이블
        self.table = QTableWidget(self)
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(['입고 ID', '상품 ID', '상품 개수', '박스 수량', '입고일시', '상태'])
        self.table.resizeColumnsToContents()
        main_layout.addWidget(self.table)
        
        self.table.setColumnWidth(0, 100)
        self.table.setColumnWidth(1, 100)
        self.table.setColumnWidth(2, 100)
        self.table.setColumnWidth(3, 100)
        self.table.setColumnWidth(4, 200)
        self.table.setColumnWidth(5, 100)

    def add_inbound_item(self):
        """
        입력된 정보를 FastAPI 서버의 /inbound 엔드포인트로 POST 요청을 보냅니다.
        """
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
                QMessageBox.information(self, '성공', f'입고 정보가 성공적으로 저장되었습니다. (입고 ID: {result["ib_id"]})')
                self.item_id_input.clear()
                self.item_amount_input.clear()
                self.ib_amount_input.clear()
                self.load_inbound_data()
            else:
                QMessageBox.warning(self, '서버 오류', '입고 정보 저장에 실패했습니다.')
                
        except ValueError as e:
            QMessageBox.warning(self, '입력 오류', f'유효하지 않은 입력입니다. 숫자를 입력해주세요. 오류: {e}')
        except requests.exceptions.RequestException as e:
            QMessageBox.critical(self, '네트워크 오류', f'서버와 통신할 수 없습니다. FastAPI 서버가 실행 중인지 확인해주세요. 오류: {e}')
        except Exception as e:
            QMessageBox.critical(self, '알 수 없는 오류', f'오류가 발생했습니다: {e}')
            print(f"Error: {e}")

    def load_inbound_data(self):
        """
        FastAPI 서버의 /inbounds 엔드포인트로 GET 요청을 보내 모든 입고 데이터를 불러옵니다.
        """
        self.table.setRowCount(0)
        try:
            response = requests.get(f"{FASTAPI_SERVER_URL}/inbounds")
            response.raise_for_status()
            inbound_items = response.json()
            
            for row_num, item in enumerate(inbound_items):
                self.table.insertRow(row_num)
                self.table.setItem(row_num, 0, QTableWidgetItem(str(item["ib_id"])))
                self.table.setItem(row_num, 1, QTableWidgetItem(str(item["item_id"])))
                self.table.setItem(row_num, 2, QTableWidgetItem(str(item["item_amount"])))
                self.table.setItem(row_num, 3, QTableWidgetItem(str(item["ib_amount"])))
                self.table.setItem(row_num, 4, QTableWidgetItem(str(item["ib_dttm"])))
                self.table.setItem(row_num, 5, QTableWidgetItem(str(item["ib_status"])))
        except requests.exceptions.RequestException as e:
            QMessageBox.critical(self, '네트워크 오류', f'데이터를 불러오는 중 서버와 통신할 수 없습니다. 오류: {e}')
        except Exception as e:
            QMessageBox.critical(self, '데이터 로드 오류', f'데이터 처리 중 오류가 발생했습니다: {e}')
            print(f"Error: {e}")

    def update_selected_inbound_status(self):
        """
        선택된 행의 입고 ID를 FastAPI 서버의 /inbound/{ib_id}/status 엔드포인트로 PATCH 요청을 보냅니다.
        """
        selected_rows = self.table.selectionModel().selectedRows()
        if not selected_rows:
            QMessageBox.warning(self, '선택 오류', '먼저 업데이트할 행을 선택해주세요.')
            return

        row_index = selected_rows[0].row()
        ib_id_item = self.table.item(row_index, 0)
        if ib_id_item:
            try:
                ib_id = int(ib_id_item.text())
                payload = {"ib_status": 1}
                response = requests.patch(f"{FASTAPI_SERVER_URL}/inbound/{ib_id}/status", json=payload)
                response.raise_for_status()
                
                result = response.json()
                if result.get("status") == "success":
                    QMessageBox.information(self, '상태 업데이트', f'입고 ID {ib_id}의 상태가 1로 업데이트되었습니다.')
                    self.load_inbound_data()
                else:
                    QMessageBox.warning(self, '업데이트 오류', f'입고 ID {ib_id}의 상태 업데이트에 실패했습니다.')
            except ValueError:
                QMessageBox.critical(self, '오류', '유효하지 않은 입고 ID입니다.')
            except requests.exceptions.RequestException as e:
                QMessageBox.critical(self, '네트워크 오류', f'상태 업데이트 중 서버와 통신할 수 없습니다. 오류: {e}')
            except Exception as e:
                QMessageBox.critical(self, '알 수 없는 오류', f'상태 업데이트 중 오류가 발생했습니다: {e}')
                print(f"Error: {e}")

    def query_inbound_by_id(self):
        """
        입고 ID를 입력받아 FastAPI 서버의 /inbound/{ib_id} 엔드포인트로 GET 요청을 보냅니다.
        """
        try:
            ib_id = int(self.ib_id_query_input.text())
            response = requests.get(f"{FASTAPI_SERVER_URL}/inbound/{ib_id}")
            response.raise_for_status()
            inbound_data = response.json()
            
            if inbound_data:
                QMessageBox.information(
                    self,
                    f'입고 ID {ib_id} 조회 결과',
                    f"입고 ID: {inbound_data['ib_id']}\n"
                    f"상품 ID: {inbound_data['item_id']}\n"
                    f"상품 개수: {inbound_data['item_amount']}\n"
                    f"박스 수량: {inbound_data['ib_amount']}\n"
                    f"상태: {inbound_data['ib_status']}\n"
                    f"입고일시: {inbound_data['ib_dttm']}"
                )
            else:
                QMessageBox.warning(self, '조회 실패', f'입고 ID {ib_id}에 해당하는 데이터가 없습니다.')
        except ValueError:
            QMessageBox.warning(self, '입력 오류', '유효하지 않은 입고 ID입니다. 숫자를 입력해주세요.')
        except requests.exceptions.RequestException as e:
            QMessageBox.critical(self, '네트워크 오류', f'데이터 조회 중 서버와 통신할 수 없습니다. 오류: {e}')
        except Exception as e:
            QMessageBox.critical(self, 'DB 오류', f'데이터 조회 중 오류가 발생했습니다: {e}')
            print(f"Error: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = InboundApp()
    ex.show()
    sys.exit(app.exec_())
