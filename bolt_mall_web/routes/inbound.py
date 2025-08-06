from fastapi import APIRouter, Depends, Request, HTTPException
from sqlalchemy.orm import Session
from database import get_db
from models import Inbound
from datetime import datetime

# create, read
router = APIRouter()

# 진열(create) - item_id, item_amount, ib_amount 받고 ib_dttm 현재 시간으로 저장
@router.post("/inbound")
async def create_inbound(request: Request, db: Session = Depends(get_db)):
    data = await request.json()

    new_inbound = Inbound(
        item_id=data["item_id"],
        item_amount=data["item_amount"],
        ib_amount=data["ib_amount"],
        ib_status=0,  # 필요에 따라 조정
        ib_dttm=datetime.now()
    )
    db.add(new_inbound)
    db.commit()
    db.refresh(new_inbound)  # 새로 생성된 객체 값을 다시 가져오기 # autoincrement된 ib_id 반영 

    return {"status": "success", "ib_id": new_inbound.ib_id}

# ib_id인 값의 상태(ib_status) 업데이트 
@router.patch("/inbound/{ib_id}/status")
async def update_ib_status(ib_id: int, request: Request, db: Session = Depends(get_db)):
    data = await request.json() # json 형식으로 ib_status 받기 
    new_status = data.get("ib_status")
    if new_status is None:
        raise HTTPException(status_code=400, detail="ib_status field is required")

    inbound = db.query(Inbound).filter(Inbound.ib_id == ib_id).first()
    if not inbound:
        raise HTTPException(status_code=404, detail="Inbound not found")

    inbound.ib_status = new_status
    db.commit()
    db.refresh(inbound)

    return {"status": "success", "ib_id": inbound.ib_id, "ib_status": inbound.ib_status}

# 단일 ib_id로 조회
@router.get("/inbound/{ib_id}")
def read_inbound(ib_id: int, db: Session = Depends(get_db)):
    inbound = db.query(Inbound).filter(Inbound.ib_id == ib_id).first()
    if not inbound:
        raise HTTPException(status_code=404, detail="Inbound not found")
    return {
        "ib_id": inbound.ib_id,
        "item_id": inbound.item_id,
        "item_amount": inbound.item_amount,
        "ib_amount": inbound.ib_amount,
        "ib_status": inbound.ib_status,
        "ib_dttm": inbound.ib_dttm.isoformat()
    }


# 전체 리스트 조회 (필요시 페이징 등 추가 가능)
@router.get("/inbounds")
def read_inbounds(db: Session = Depends(get_db)):
    inbounds = db.query(Inbound).all()
    results = []
    for inbound in inbounds:
        results.append({
            "ib_id": inbound.ib_id,
            "item_id": inbound.item_id,
            "item_amount": inbound.item_amount,
            "ib_amount": inbound.ib_amount,
            "ib_status": inbound.ib_status,
            "ib_dttm": inbound.ib_dttm.isoformat()
        })
    return results