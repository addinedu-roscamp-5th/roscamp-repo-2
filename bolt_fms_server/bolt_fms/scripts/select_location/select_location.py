# -*- coding: utf-8 -*-
"""
'models.py'와 'database.py'를 import하여 입고 위치를 찾고 로봇의 좌표를 계산하는 스크립트입니다.
"""
from sqlalchemy.orm import Session
from sqlalchemy import select
from database import SessionLocal
from models import Rack, Location, Inventory

# 1. 비어있는 가장 낮은 번호의 location_id를 찾는 함수
def get_next_available_location_id(db: Session) -> int | None:
    """
    Inventory에 등록되지 않은 location_id 중 가장 작은 값을 찾아서 반환합니다.

    Args:
        db: SQLAlchemy 세션 객체.

    Returns:
        가장 낮은 번호의 사용 가능한 location_id 또는 없으면 None.
    """
    # select()를 명시적으로 사용하여 SQLAlchemy 2.0 권장 문법을 따릅니다.
    subquery = select(Inventory.location_id)
    result = db.query(Location.location_id)\
        .filter(Location.location_id.notin_(subquery))\
        .order_by(Location.location_id.asc())\
        .first()

    return result[0] if result else None


# 2. 로봇의 최종 x, y 좌표를 계산하는 함수
def get_robot_coordinates(db: Session, location_id: int) -> tuple[float, float] | None:
    """
    주어진 location_id에 대한 로봇의 최종 x, y 좌표를 계산합니다.

    Args:
        db: SQLAlchemy 세션 객체.
        location_id: 계산할 위치의 ID.

    Returns:
        (x, y) 좌표 튜플 또는 해당 위치를 찾을 수 없는 경우 None.
    """
    location_data = db.query(Location, Rack)\
        .join(Rack, Location.rack == Rack.rack)\
        .filter(Location.location_id == location_id)\
        .first()

    if not location_data:
        print(f"오류: Location ID {location_id}에 대한 정보를 찾을 수 없습니다.")
        return None

    location, rack = location_data

    # col_num과 cell_size를 활용하여 y 좌표 계산
    y_col_offset = float(location.col_num - 1) * float(rack.cell_size)
    y_offset = 4
    y_coord = float(rack.y_start) + y_col_offset + 4 # y 좌표는 col_num에 따라 계산 + 4cm (중간으로 이동)

    # 랙의 방향에 따라 x 좌표 계산
    x_diff = float(rack.x_start) - float(rack.x_end) 
    x_offset = 16

    if x_diff < 0:
        x_coord = float(rack.x_start) - x_offset
    else:
        x_coord = float(rack.x_start) + x_offset
        
    return {
        "x": x_coord,
        "y": y_coord,
        "rack": location.rack,
        "row_num": location.row_num,
        "col_num": location.col_num,
        "location_id": location.location_id
    }   


# 3. 전체 로직 실행 예시
def main():
    """
    전체 로직을 실행하는 메인 함수입니다.
    """
    db = SessionLocal()
    try:
        # 1단계: 다음 입고할 위치(location_id) 찾기
        next_location_id = get_next_available_location_id(db)

        if next_location_id:
            print(f"✨ 다음으로 입고할 위치는 location_id: {next_location_id} 입니다.")

            # 2단계: 해당 location_id에 대한 로봇 좌표 계산
            # location_info = get_robot_coordinates(db, next_location_id)
            location_info = get_robot_coordinates(db, 7)

            if location_info:
                print(f"--- 상세 정보 ---")
                print(f"➡️ Location ID: {location_info['location_id']}")
                print(f"➡️ Rack 번호: {location_info['rack']}")
                print(f"➡️ 선반 행/열 번호: {location_info['row_num']}행, {location_info['col_num']}열")
                print(f"--- 최종 로봇 좌표 ---")
                print(f"📦 (x: {location_info['x']}, y: {location_info['y']})")
            else:
                print("좌표를 계산할 수 없습니다.")
        else:
            print("📦 현재 입고 가능한 빈 위치가 없습니다.")

    except Exception as e:
        print(f"⛔️ 오류가 발생했습니다: {e}")
    finally:
        db.close()

if __name__ == "__main__":
    main()

