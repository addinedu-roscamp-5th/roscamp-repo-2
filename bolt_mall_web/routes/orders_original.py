from fastapi import APIRouter, Depends, Request
from database import get_db
from databases import Database

router = APIRouter()

c_id = 0  # 임시 고객번호

@router.post("/orders")
async def create_order(request: Request, db: Database = Depends(get_db)):
    global c_id
    c_id += 1  # 임시 고객번호 증가

    data = await request.json()
    # print("받은 주문 데이터:", data)   # 서버 터미널에 출력됨
    items = data.get("items", [])
    # print("상품 리스트 개수:", len(items))

    # 1) Orders 테이블에 주문 기본 데이터 삽입 (고객ID, 시간, 상태)
    query_order = """
    INSERT INTO Orders (customer_id, order_dttm, order_status)
    VALUES (:customer_id, NOW(), 0)
    """
    order_id = await db.execute(query=query_order, values={"customer_id": c_id})

    # 2) Orders_Item 테이블에 주문한 상품들 각각 저장
    for item in items:
        query_item = """
        INSERT INTO Orders_Item (order_id, item_id, order_amount, unit_price)
        VALUES (:order_id, :item_id, :order_amount, :unit_price)
        """
        await db.execute(query=query_item, values={
            "order_id": order_id,
            "item_id": item["item_id"],
            "order_amount": item["order_amount"],
            "unit_price": item["unit_price"]
        })

    return {"status": "success"}
