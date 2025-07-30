# database.py
from sqlalchemy import create_engine, MetaData
from databases import Database

# ⚙️ DB 연결 정보 (환경변수나 config에서 불러오는 것이 좋음)
DATABASE_URL = "mysql+pymysql://eunyoung:password@192.168.0.139:3306/BoltDB"

# SQLAlchemy 설정
engine = create_engine(DATABASE_URL)
metadata = MetaData()

# databases 패키지로 연결
database = Database(DATABASE_URL)

# ✅ FastAPI 의 Depends에서 사용될 비동기 의존성 함수
async def get_db():
    if not database.is_connected:
        await database.connect()
    try:
        yield database
    finally:
        await database.disconnect()