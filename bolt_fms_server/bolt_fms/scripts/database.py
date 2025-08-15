from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base

# 1. 데이터베이스 접속 정보
DATABASE_URL = "mysql+pymysql://eunyoung:password@192.168.0.139:3306/BoltDB"

# 2. SQLAlchemy 엔진 생성
engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,      # 연결 풀에서 자동 재연결
    pool_recycle=3600,       # 연결 재활용 설정(1시간)
    echo=False               # 로그 필요시 True
)

# 3. 세션 팩토리 생성
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 4. Base 클래스 선언 (models.py에서 모델이 이걸 상속)
Base = declarative_base()

# 5. FastAPI Depends 의존성 함수
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()