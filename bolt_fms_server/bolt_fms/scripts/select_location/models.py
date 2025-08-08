from sqlalchemy import Column, Integer, String, DECIMAL, DateTime, ForeignKey, CheckConstraint
from sqlalchemy.orm import relationship
from database import Base

# Item (상품 설명)
class Item(Base):
    __tablename__ = "Item"

    item_id = Column(Integer, primary_key=True, nullable=False)
    item_name = Column(String(20), nullable=False)
    category = Column(String(20), nullable=False)
    price = Column(DECIMAL(10, 2), nullable=False)
    dimension_x = Column(DECIMAL(10, 2), nullable=False)
    dimension_y = Column(DECIMAL(10, 2), nullable=False)
    dimension_z = Column(DECIMAL(10, 2), nullable=False)

    __table_args__ = (
        CheckConstraint('price >= 0', name='check_price_nonnegative'),
    )

    # 관계 설정
    inventories = relationship("Inventory", back_populates="item_relation")
    orders_items = relationship("Orders_Item", back_populates="item_relation")
    inbounds = relationship("Inbound", back_populates="item_relation")

# Rack (랙)
class Rack(Base):
    __tablename__ = "Rack"

    rack = Column(Integer, primary_key=True, nullable=False)
    x_start = Column(DECIMAL(10, 2), nullable=False)
    x_end = Column(DECIMAL(10, 2), nullable=False)
    y_start = Column(DECIMAL(10, 2), nullable=False)
    y_end = Column(DECIMAL(10, 2), nullable=False)
    cell_size = Column(Integer, nullable=False)

    __table_args__ = (
        CheckConstraint('x_start >= 0', name='check_x_start_nonnegative'),
        CheckConstraint('x_end >= 0', name='check_x_end_nonnegative'),
        CheckConstraint('y_start >= 0', name='check_y_start_nonnegative'),
        CheckConstraint('y_end >= 0', name='check_y_end_nonnegative'),
    )

    locations = relationship("Location", back_populates="rack_relation")

# Location (적재 위치)
class Location(Base):
    __tablename__ = "Location"

    location_id = Column(Integer, primary_key=True, autoincrement=True)
    rack = Column(Integer, ForeignKey("Rack.rack"), nullable=False)
    row_num = Column(Integer, nullable=False)
    col_num = Column(Integer, nullable=False)
    description = Column(String(100), nullable=True)

    rack_relation = relationship("Rack", back_populates="locations")
    inventories = relationship("Inventory", back_populates="location_relation")

# Inventory (재고 위치)
class Inventory(Base):
    __tablename__ = "Inventory"

    location_id = Column(Integer, ForeignKey("Location.location_id"), primary_key=True, nullable=False)
    item_id = Column(Integer, ForeignKey("Item.item_id"), primary_key=True, nullable=False)
    iv_dttm = Column(DateTime, nullable=False)

    location_relation = relationship("Location", back_populates="inventories")
    item_relation = relationship("Item", back_populates="inventories")

# Orders (주문 정보)
class Orders(Base):
    __tablename__ = "Orders"

    order_id = Column(Integer, primary_key=True, autoincrement=True)
    customer_id = Column(Integer, nullable=False)  # FK는 나중에 Customer 테이블 생기면 추가 가능
    order_dttm = Column(DateTime, nullable=False)
    order_status = Column(Integer, nullable=False)
    destination = Column(String(40), nullable=True)

    order_items = relationship("Orders_Item", back_populates="order_relation")
    outbounds = relationship("Outbound", back_populates="order_relation")

# Orders_Item (각 주문별 상품)
class Orders_Item(Base):
    __tablename__ = "Orders_Item"

    order_id = Column(Integer, ForeignKey("Orders.order_id"), primary_key=True, nullable=False)
    item_id = Column(Integer, ForeignKey("Item.item_id"), primary_key=True, nullable=False)
    order_amount = Column(Integer, nullable=False)
    unit_price = Column(DECIMAL(10, 2), nullable=False)

    __table_args__ = (
        CheckConstraint('order_amount > 0', name='check_order_amount_positive'),
        CheckConstraint('unit_price >= 0', name='check_unit_price_nonnegative'),
    )

    order_relation = relationship("Orders", back_populates="order_items")
    item_relation = relationship("Item", back_populates="orders_items")

# Outbound (출고)
class Outbound(Base):
    __tablename__ = "Outbound"

    order_id = Column(Integer, ForeignKey("Orders.order_id"), primary_key=True, nullable=False)
    ob_id = Column(Integer, primary_key=True, nullable=False)
    ob_dttm = Column(DateTime, nullable=False)
    ob_status = Column(Integer, nullable=False)
    destination = Column(String(40), nullable=True)

    order_relation = relationship("Orders", back_populates="outbounds")


# Inbound (입고)
class Inbound(Base):
    __tablename__ = "Inbound"

    ib_id = Column(Integer, primary_key=True, autoincrement=True)
    item_id = Column(Integer, ForeignKey("Item.item_id"), nullable=False)
    item_amount = Column(Integer, nullable=False)
    ib_amount = Column(Integer, nullable=False)
    ib_status = Column(Integer, nullable=False)
    ib_dttm = Column(DateTime, nullable=False)

    item_relation = relationship("Item", back_populates="inbounds")
