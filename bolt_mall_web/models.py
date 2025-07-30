from sqlalchemy import Column, Integer, String, DateTime, ForeignKey, DECIMAL
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class Order(Base):
    __tablename__ = "Order"
    order_id = Column(Integer, primary_key=True, autoincrement=True)
    customer_id = Column(Integer, nullable=False)
    order_date = Column(DateTime, nullable=False)

class OrderItem(Base):
    __tablename__ = "Order_Item"
    order_item_id = Column(Integer, primary_key=True, autoincrement=True)
    order_id = Column(Integer, ForeignKey("Order.order_id"), nullable=False)
    customer_id = Column(Integer, nullable=False)
    item_id = Column(Integer, ForeignKey("Item.item_id"), nullable=False)
    quantity = Column(Integer, nullable=False)
    unit_price = Column(DECIMAL(10, 2), nullable=False)
    status = Column(String(20), nullable=False)
    order_date = Column(DateTime, nullable=False)
