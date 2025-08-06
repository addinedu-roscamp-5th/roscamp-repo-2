from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session
from database import get_db
from models import Outbound, Orders
from datetime import datetime

router = APIRouter()

# 출고(create) - order_id, ob_id 받고 ob_dttm 현재 시간으로 저장
@router.post("/outbound")
async def create_outbound(request: Request, db: Session = Depends(get_db)):
    data = await request.json()
    order_id = data.get("order_id")
    ob_id = data.get("ob_id")
    
    if order_id is None or ob_id is None:
        raise HTTPException(status_code=400, detail="order_id and ob_id are required")

    # 1) 주문 상태 체크 (예: order_status가 특정 값이어야 출고 가능)
    order = db.query(Orders).filter(Orders.order_id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    
    # 주문 테이블의 상태(order_status)가 1(완료)일 때만 출고 생성 가능 (조건은 실제 요구사항에 맞게 변경)
    ALLOWED_STATUS = 1
    if order.order_status != ALLOWED_STATUS:
        raise HTTPException(status_code=400, detail=f"Order status must be {ALLOWED_STATUS} to create outbound")

    # 2) Outbound 레코드 생성
    new_outbound = Outbound(
        order_id=order_id,
        ob_id=ob_id,
        ob_status=0,          # 기본값 0
        ob_dttm=datetime.now(),
        destination=None      # 우선 널값
    )
    db.add(new_outbound)
    try:
        db.commit()
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to create Outbound: {str(e)}")

    return {
        "status": "success",
        "order_id": new_outbound.order_id,
        "ob_id": new_outbound.ob_id,
        "ob_status": new_outbound.ob_status,
        "ob_dttm": new_outbound.ob_dttm.isoformat(),
        "destination": new_outbound.destination,
    }

# ob_id에 해당하는 outbound 레코드의 ob_status 업데이트
@router.patch("/outbound/{ob_id}/status")
async def update_ob_status(ob_id: int, request: Request, db: Session = Depends(get_db)):
    data = await request.json()
    new_status = data.get("ob_status")
    if new_status is None:
        raise HTTPException(status_code=400, detail="ob_status field is required")

    outbound = db.query(Outbound).filter(Outbound.ob_id == ob_id).first()
    if not outbound:
        raise HTTPException(status_code=404, detail="Outbound not found")

    outbound.ob_status = new_status
    db.commit()
    db.refresh(outbound)

    return {
        "status": "success",
        "order_id": outbound.order_id,
        "ob_id": outbound.ob_id,
        "ob_status": outbound.ob_status
    }

# 단일 ob_id 조회
@router.get("/outbound/{ob_id}")
def read_outbound(ob_id: int, db: Session = Depends(get_db)):
    outbound = db.query(Outbound).filter(Outbound.ob_id == ob_id).first()
    if not outbound:
        raise HTTPException(status_code=404, detail="Outbound not found")
    return {
        "order_id": outbound.order_id,
        "ob_id": outbound.ob_id,
        "ob_status": outbound.ob_status,
        "ob_dttm": outbound.ob_dttm.isoformat(),
        "destination": outbound.destination
    }


# 전체 Outbound 리스트 조회
@router.get("/outbounds")
def read_outbounds(db: Session = Depends(get_db)):
    outbounds = db.query(Outbound).all()
    results = []
    for outbound in outbounds:
        results.append({
            "order_id": outbound.order_id,
            "ob_id": outbound.ob_id,
            "ob_status": outbound.ob_status,
            "ob_dttm": outbound.ob_dttm.isoformat(),
            "destination": outbound.destination
        })
    return results