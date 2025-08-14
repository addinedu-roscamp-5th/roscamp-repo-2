import os
import sys
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QTextEdit, QLineEdit, QPushButton, QLabel)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from sqlalchemy import create_engine, inspect, text
from sqlalchemy.orm import Session
import google.generativeai as genai
import pandas as pd
from datetime import datetime, timedelta
import markdown
from models import Base, Rack, Location, Inventory, Item, Orders, Orders_Item, Outbound, Inbound

# --- Gemini API 설정 ---
GOOGLE_API_KEY = "YOUR_API_KEY"  # 실제 API 키로 변경하세요
if not GOOGLE_API_KEY:
    raise ValueError("GEMINI_API_KEY 환경 변수가 설정되지 않았습니다.")

genai.configure(api_key=GOOGLE_API_KEY)
MODEL_NAME = "gemini-1.5-flash-latest"

# --- SQLAlchemy 설정 ---
DATABASE_URL = "mysql+pymysql://eunyoung:password@192.168.0.139:3306/BoltDB"
engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,
    pool_recycle=3600,
)
SessionLocal = Session(bind=engine)

# --- 백그라운드 스레드: Gemini API 호출 ---
class GeminiWorker(QThread):
    finished = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, prompt_text):
        super().__init__()
        self.prompt_text = prompt_text

    def run(self):
        try:
            model = genai.GenerativeModel(MODEL_NAME)
            response = model.generate_content(self.prompt_text)
            self.finished.emit(response.text)
        except Exception as e:
            self.error.emit(f"Gemini API 호출 중 오류 발생: {e}")

# --- DB 스키마 정보 가져오기 ---
def get_db_schema_info():
    # """DB의 모든 테이블 스키마 정보를 문자열로 반환합니다."""
    # inspector = inspect(engine)
    # table_names = inspector.get_table_names()

    # schema_info = "## 데이터베이스 테이블 스키마 정보\n\n"
    # for table_name in table_names:
    #     schema_info += f"### 테이블: {table_name}\n"
    #     columns = inspector.get_columns(table_name)
    #     schema_info += "| 컬럼명 | 데이터 타입 | Nullable |\n"
    #     schema_info += "|---|---|---|\n"
    #     for col in columns:
    #         schema_info += f"| {col['name']} | {col['type']} | {'Yes' if col['nullable'] else 'No'} |\n"
    #     schema_info += "\n"

    """제공된 상세한 DB 스키마 정보를 문자열로 반환합니다."""
    schema_info = """
    ## 데이터베이스 테이블 스키마 정보

    ### 1. Item (상품 설명)
    * **item_id**: INT, PRIMARY KEY, NOT NULL. 상품 고유번호. April tag 번호와 동일합니다.
    * **item_name**: VARCHAR(20), NOT NULL. 상품명.
    * **category**: VARCHAR(20), NOT NULL. 카테고리.
    * **price**: DECIMAL(10,2), NOT NULL. 단가.
    * **dimension_x**: DECIMAL(10,2), NOT NULL. 가로 길이.
    * **dimension_y**: DECIMAL(10,2), NOT NULL. 세로 길이.
    * **dimension_z**: DECIMAL(10,2), NOT NULL. 높이.

    ### 2. Rack (랙 위치)
    * **rack**: INT, PRIMARY KEY, NOT NULL. 랙 번호.
    * **x_start**: DECIMAL(10,2), NOT NULL. 랙의 x축 시작 좌표.
    * **x_end**: DECIMAL(10,2), NOT NULL. 랙의 x축 끝 좌표.
    * **y_start**: DECIMAL(10,2), NOT NULL. 랙의 y축 시작 좌표.
    * **y_end**: DECIMAL(10,2), NOT NULL. 랙의 y축 끝 좌표.
    * **cell_size**: INT, NOT NULL. 랙의 한 칸의 크기.

    ### 3. Location (적재 위치)
    * **location_id**: INT, PRIMARY KEY, AUTO_INCREMENT. 위치 고유번호.
    * **rack**: INT, NOT NULL, FOREIGN KEY. 랙 번호.
    * **row_num**: INT, NOT NULL. 선반 번호 (행).
    * **col_num**: INT, NOT NULL. 선반의 가로 위치 (열).
    * **description**: VARCHAR(100), NULL 허용. 위치에 대한 추가 설명.

    ### 4. Inventory (각 Item의 Location 정보)
    * **location_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 상품이 위치한 칸의 고유번호.
    * **item_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 상품 ID.
    * **iv_dttm**: DATETIME, NOT NULL. 재고가 쌓인 날짜와 시간.
    * **중요 규칙**: Inventory 테이블의 각 행은 재고에 있는 **상품 하나**를 의미합니다. 재고 수량은 이 테이블의 행 개수로 계산해야 합니다.

    ### 5. Orders (주문 정보)
    * **order_id**: INT, PRIMARY KEY, AUTO_INCREMENT. 주문 고유번호.
    * **customer_id**: INT, NOT NULL. 고객 고유번호.
    * **order_dttm**: DATETIME, NOT NULL. 주문일자 및 시간.
    * **order_status**: INT, NOT NULL. 주문 상태 (0: 집품대기, 1: 집품중, 2: 집품완료, 3: 출고대기).
    * **destination**: VARCHAR(40), NULL 허용. 배송지.

    ### 6. Orders_Item (각 주문별 상품)
    * **복합키**: (order_id, item_id)가 기본 키입니다.
    * **order_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 주문 ID.
    * **item_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 상품 ID.
    * **order_amount**: INT, NOT NULL. 주문 수량.
    * **unit_price**: DECIMAL(10,2), NOT NULL. 주문 당시 단가.

    ### 7. Outbound (출고 관련)
    * **order_id**: INT, PRIMARY KEY, NOT NULL, FOREIGN KEY. 주문 고유번호.
    * **ob_id**: INT, PRIMARY KEY, NOT NULL. 출고 바코드 번호 (포장 시 Apriltag 번호).
    * **ob_dttm**: DATETIME, NOT NULL. 출고일자 및 시간.
    * **ob_status**: INT, NOT NULL. 출고 상태 (0: 출고전, 1: 출고완료).
    * **destination**: VARCHAR(40), NULL 허용. 배송지.

    ### 8. Inbound (입고 관련)
    * **ib_id**: INT, PRIMARY KEY, AUTO_INCREMENT. 입고 ID.
    * **ib_amount**: INT, NOT NULL. 입고된 박스 수량.
    * **item_id**: INT, NOT NULL, FOREIGN KEY. 상품 ID.
    * **item_amount**: INT, NOT NULL. 한 박스에 들어있는 상품 개수.
    * **ib_dttm**: DATETIME, NOT NULL. 입고일자 및 시간.
    * **ib_status**: INT, NOT NULL. 입고 진행 상황.
    """
    return schema_info

# --- DB에서 실제 데이터 조회 ---
def get_db_data_for_rag(query_keywords):
    """
    모든 테이블의 데이터를 조회하여 문자열로 반환합니다.
    """
    related_data_info = ""
    with engine.connect() as connection:
        # models.py에 정의된 모든 테이블을 가져옵니다.
        tables = Base.metadata.tables
        
        for table_name, table in tables.items():
            try:
                sql_query = text(f"SELECT * FROM {table_name}")
                result = connection.execute(sql_query)
                df = pd.DataFrame(result.fetchall(), columns=result.keys())
                
                if not df.empty:
                    related_data_info += f"### 테이블 `{table_name}`의 전체 데이터\n"
                    related_data_info += df.to_markdown(index=False)
                    related_data_info += "\n\n"
            except Exception as e:
                print(f"Error fetching data from {table_name}: {e}")
                continue
                
    return related_data_info

# --- QT 애플리케이션 ---
class RAGChatbot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.db_schema = get_db_schema_info()
        self.gemini_worker = None

    def initUI(self):
        self.setWindowTitle('물류 창고 RAG 챗봇 (Gemini 1.5 Flash)')
        self.setGeometry(300, 300, 600, 500)

        main_layout = QVBoxLayout()

        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.chat_display.setStyleSheet("font-size: 14px; padding: 10px; background-color: #f0f0f0;")
        main_layout.addWidget(self.chat_display)

        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText('질문을 입력하세요...')
        self.input_field.returnPressed.connect(self.send_message)
        self.input_field.setStyleSheet("font-size: 14px; padding: 5px;")
        main_layout.addWidget(self.input_field)

        self.send_button = QPushButton('전송')
        self.send_button.clicked.connect(self.send_message)
        self.send_button.setStyleSheet("font-size: 14px; padding: 5px;")
        main_layout.addWidget(self.send_button)
        
        self.status_label = QLabel("준비 완료")
        self.status_label.setStyleSheet("font-size: 12px; color: gray;")
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)

    def send_message(self):
            user_question = self.input_field.text()
            if not user_question.strip():
                return
            
            self.chat_display.append(f"<p style='color: #007bff;'><b>나:</b> {user_question}</p>")
            self.input_field.clear()
            
            self.input_field.setEnabled(False)
            self.send_button.setEnabled(False)
            self.status_label.setText("데이터 조회 및 답변 생성 중...")

            # 1. 오늘 날짜 정보 가져오기
            today_date = datetime.now().strftime("%Y년 %m월 %d일")
            
            # 2. 키워드를 기반으로 DB에서 관련 데이터 조회
            db_data = get_db_data_for_rag(user_question)
            
            # 3. DB 스키마, 현재 날짜, 실제 데이터를 포함한 RAG 프롬프트 구성
            prompt_template = (
                "당신은 물류 창고 관리 시스템의 AI 챗봇입니다. "
                "사용자가 요청하는 데이터가 있을 경우, 아래 제공된 데이터베이스 스키마와 실제 데이터를 바탕으로 답변하세요. "
                "답변할 때는 친절하고, 명확하며, 상세하게 설명해주세요. "
                "특히, **아래에 제공된 '현재 날짜' 정보를 참고**하여 '오늘'이나 '어제'와 같은 시간 관련 질문에 답변해주세요."
                "데이터가 존재하지 않거나 질문에 대한 정보가 부족할 경우, '죄송합니다. 현재 데이터에 요청하신 정보가 없습니다.'와 같이 정중하게 답하고, "
                "어떤 데이터가 필요한지(예: 특정 날짜의 주문 정보)를 구체적으로 알려주세요. "
                "답변은 가독성을 높이기 위해 불릿 포인트(`*` 또는 `-`)를 활용하여 정리해주세요.\n\n"
                
                f"{self.db_schema}\n\n"
                f"## 현재 날짜 정보\n"
                f"현재 날짜는 {today_date}입니다.\n\n"
                f"## 데이터베이스에서 조회한 실제 데이터\n\n"
                f"{db_data}"
                f"사용자 질문: {user_question}\n\n"
                "답변: "
            )

            # Gemini API 호출을 위한 스레드 시작
            self.gemini_worker = GeminiWorker(prompt_template)
            self.gemini_worker.finished.connect(self.handle_response)
            self.gemini_worker.error.connect(self.handle_error)
            self.gemini_worker.start()

    def handle_response(self, response_text):
        # Gemini의 마크다운 답변을 HTML로 변환
        html_response = markdown.markdown(response_text)
        
        # HTML을 `QTextEdit`에 추가 (append 대신 setHtml을 사용하면 기존 내용을 덮어씁니다)
        self.chat_display.append(f"<p><b>Gemini:</b> {html_response}</p>")
        self.reset_ui()

    def handle_error(self, error_message):
        self.chat_display.append(f"<p style='color: red;'><b>오류:</b> {error_message}</p>")
        self.reset_ui()

    
    def reset_ui(self):
        self.input_field.setEnabled(True)
        self.send_button.setEnabled(True)
        self.status_label.setText("준비 완료")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    chat_window = RAGChatbot()
    chat_window.show()
    sys.exit(app.exec_())