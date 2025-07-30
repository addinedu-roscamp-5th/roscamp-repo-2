from fastapi import APIRouter, Depends, Request
from database import get_db
from databases import Database

router = APIRouter()

c_id = 0
@router.post("/orders")
async def create_order(request: Request, db: Database = Depends(get_db)):
    data = await request.json()
    items = data.get("items", [])
    c_id = c_id+1
    for item in items:
        await db.execute(
            """
            INSERT INTO Order_Item (customer_id, item_id, quantity, unit_price, status, order_date)
            VALUES (:customer_id, :item_id, :quantity, :unit_price, :status, NOW())
            """,
            {
                "customer_id": c_id,  # 임시값 또는 인증 사용자 기반 ID
                "item_id": item["itemId"],
                "quantity": item["quantity"],
                "unit_price": item["price"],
                "status": "대기"
                
            }
        )

    return { "status": "success" }
