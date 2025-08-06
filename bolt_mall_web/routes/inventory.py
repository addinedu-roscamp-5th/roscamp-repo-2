from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session
from database import get_db
from models import Inventory
from datetime import datetime

router = APIRouter()

# 진열(create) - location_id, item_id 받고 iv_dttm 현재 시간으로 저장
@router.post("/inventory")
async def create_inventory(request: Request, db: Session = Depends(get_db)):
    data = await request.json()
    location_id = data.get("location_id")
    item_id = data.get("item_id")

    if location_id is None or item_id is None:
        raise HTTPException(status_code=400, detail="location_id and item_id are required")

    # 이미 같은 (location_id, item_id) 조합이 존재하면 중복 방지
    existing = db.query(Inventory).filter(
        Inventory.location_id == location_id,
        Inventory.item_id == item_id
    ).first()
    if existing:
        raise HTTPException(status_code=400, detail="Inventory entry already exists for given location_id and item_id")

    new_inventory = Inventory(
        location_id=location_id,
        item_id=item_id,
        iv_dttm=datetime.now()
    )
    db.add(new_inventory)
    try: # 혹시나 예외가 발생할 경우를 대비해 트랜잭션 처리
        db.commit()
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to create inventory: {str(e)}")

    return {
        "status": "success",
        "location_id": location_id,
        "item_id": item_id,
        "iv_dttm": new_inventory.iv_dttm.isoformat()
    }

# 출고(delete) - location_id, item_id로 행 삭제
@router.delete("/inventory")
async def delete_inventory(request: Request, db: Session = Depends(get_db)):
    data = await request.json()
    location_id = data.get("location_id")
    item_id = data.get("item_id")

    if location_id is None or item_id is None:
        raise HTTPException(status_code=400, detail="location_id and item_id are required")

    inventory = db.query(Inventory).filter(
        Inventory.location_id == location_id,
        Inventory.item_id == item_id
    ).first()

    if not inventory:
        raise HTTPException(status_code=404, detail="Inventory entry not found")

    db.delete(inventory)
    try:
        db.commit()
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to delete inventory: {str(e)}")

    return {
        "status": "success",
        "message": f"Inventory entry at location_id {location_id} for item_id {item_id} deleted"
    }

# 전체 Inventory 리스트 조회
@router.get("/inventories")
def read_inventories(db: Session = Depends(get_db)):
    inventories = db.query(Inventory).all()
    results = []
    for inv in inventories:
        results.append({
            "location_id": inv.location_id,
            "item_id": inv.item_id,
            "iv_dttm": inv.iv_dttm.isoformat()
        })
    return results

# item_id 기준으로 Inventory 조회
@router.get("/inventories/item/{item_id}")
def read_inventory_by_item(item_id: int, db: Session = Depends(get_db)):
    inventories = db.query(Inventory).filter(Inventory.item_id == item_id).all()
    if not inventories:
        return []  # 없으면 빈 리스트 반환
    results = []
    for inv in inventories:
        results.append({
            "location_id": inv.location_id,
            "item_id": inv.item_id,
            "iv_dttm": inv.iv_dttm.isoformat()
        })
    return results