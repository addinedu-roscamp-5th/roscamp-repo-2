# -*- coding: utf-8 -*-
"""
'models.py'와 'database.py'를 import하여 입고 위치를 찾고 로봇의 좌표를 계산하는 스크립트입니다.
"""
from sqlalchemy.orm import Session
from sqlalchemy import select
from database import SessionLocal
from models import Rack, Location, Inventory
from typing import Any

# 1. 비어있는 가장 낮은 번호의 location_id를 찾는 함수
def _get_next_available_location_id(db: Session) -> int | None:
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
def _get_robot_coordinates(db: Session, location_id: int) -> dict[str, Any] | None:
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
    y_coord = float(rack.y_start) + y_col_offset + y_offset # y 좌표는 col_num에 따라 계산 + 4cm (중간으로 이동)

    # 랙의 방향에 따라 x 좌표 계산
    x_diff = float(rack.x_start) - float(rack.x_end) 
    x_offset = 16

    if x_diff < 0:
        x_coord = float(rack.x_start) - x_offset
        yaw = 180  
    else:
        x_coord = float(rack.x_start) + x_offset
        yaw = 0
        
    return {
        "x": x_coord,
        "y": y_coord,
        "yaw": yaw,
        "rack": location.rack,
        "row_num": location.row_num,
        "col_num": location.col_num,
        "location_id": location.location_id
    }   

def find_next_robot_coordinates() -> dict[str, Any] | None:
    """
    다음 입고할 위치와 해당 로봇 좌표를 한 번에 계산.
    외부에서 인자 없이 호출 가능.
    """
    db = SessionLocal()
    try:
        next_location_id = _get_next_available_location_id(db)
        if not next_location_id:
            print("📦 현재 입고 가능한 빈 위치가 없습니다.")
            return None
        return _get_robot_coordinates(db, next_location_id)
    except Exception as e:
        print(f"⛔ 오류 발생: {e}")
        return None
    finally:
        db.close()


# 3. 전체 로직 실행 예시
if __name__ == "__main__":
    coords = find_next_robot_coordinates()
    if coords:
        print(f"➡️  Location ID: {coords['location_id']}")
        print(f"➡️  Rack 번호: {coords['rack']}")
        print(f"➡️  행/열: {coords['row_num']}행, {coords['col_num']}열")
        print(f"📍 로봇 좌표: (x={coords['x']}, y={coords['y']}, yaw={coords['yaw']})")

# # Example usage:
# from select_location import find_next_robot_coordinates
# coords = find_next_robot_coordinates()