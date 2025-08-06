from database import engine, Base
import models  # models 모듈을 임포트해야 Base에 등록된 모든 모델이 로드됨

def create_tables():
    Base.metadata.create_all(bind=engine)
    print("테이블 생성 완료")

if __name__ == "__main__":
    create_tables()

