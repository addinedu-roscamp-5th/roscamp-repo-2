from fastapi import APIRouter, Depends, Request, HTTPException
from sqlalchemy.orm import Session
from database import get_db
from models import Orders, Orders_Item  # models.py에 정의한 클래스 import
from datetime import datetime

router = APIRouter()

# 주문이 들어왔을 때 Orders 테이블에 저장하고, Orders_Item 테이블에 주문 상품들을 저장
c_id = 0  # 임시 고객번호
@router.post("/orders") # /orders에 POST 요청이 들어오면 실행
async def create_order(request: Request, db: Session = Depends(get_db)):
    global c_id
    c_id += 1

    data = await request.json()
    items = data.get("items", [])

    # 1) Orders 테이블에 주문 데이터 삽입
    new_order = Orders(
        customer_id=c_id,
        order_dttm=datetime.now(),
        order_status=0
    )
    db.add(new_order)
    db.flush()  # 먼저 flush해서 new_order.order_id 값 획득

    # 2) 주문 상품들 각각 저장
    for item in items:
        order_item = Orders_Item(
            order_id=new_order.order_id,
            item_id=item["item_id"],
            order_amount=item["order_amount"],
            unit_price=item["unit_price"]
        )
        db.add(order_item)

    db.commit()  # 최종 commit

    return {"status": "success", "order_id": new_order.order_id}

# 주문 상품 조회
@router.get("/orders/{order_id}/items")
def read_order_item(order_id: int, db: Session = Depends(get_db)):
    # 해당 order_id에 속한 주문 상품들 조회
    order_items = db.query(Orders_Item).filter(Orders_Item.order_id == order_id).all()

    if not order_items:
        raise HTTPException(status_code=404, detail="Order items not found")

    # 필요한 필드만 추출해서 반환
    results = [
        {
            "item_id": order_item.item_id,
            "order_amount": order_item.order_amount
        }
        for order_item in order_items
    ]

    return {"order_id": order_id, "items": results}
