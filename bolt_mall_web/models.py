from sqlalchemy import Column, Integer, String, DECIMAL, DateTime, ForeignKey, CheckConstraint
from sqlalchemy.orm import declarative_base, relationship

Base = declarative_base()

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


class Rack(Base):
    __tablename__ = "Rack"
    rack = Column(Integer, primary_key=True, nullable=False)  # 랙 번호
    x_start = Column(Integer, nullable=False)
    x_end = Column(Integer, nullable=False)
    y_start = Column(Integer, nullable=False)
    y_end = Column(Integer, nullable=False)
    cell_size = Column(Integer, nullable=False)

    __table_args__ = (
        CheckConstraint('x_start >= 0', name='check_x_start_nonnegative'),
        CheckConstraint('x_end >= 0', name='check_x_end_nonnegative'),
        CheckConstraint('y_start >= 0', name='check_y_start_nonnegative'),
        CheckConstraint('y_end >= 0', name='check_y_end_nonnegative'),
    )


class Location(Base):
    __tablename__ = "Location"
    location_id = Column(Integer, primary_key=True, autoincrement=True)
    rack = Column(Integer, ForeignKey("Rack.rack"), nullable=False)
    row_num = Column(Integer, nullable=False)
    col_num = Column(Integer, nullable=False)
    description = Column(String(100), nullable=True)  # NULL 허용

    rack_relation = relationship("Rack", backref="locations")


class Inventory(Base):
    __tablename__ = "Inventory"
    inventory_id = Column(Integer, primary_key=True, autoincrement=True)
    location_id = Column(Integer, ForeignKey("Location.location_id"), nullable=False)
    item_id = Column(Integer, ForeignKey("Item.item_id"), nullable=False)
    ib_dttm = Column(DateTime, nullable=False)
    amount = Column(Integer, nullable=False)

    __table_args__ = (
        CheckConstraint('amount >= 0', name='check_amount_nonnegative'),
    )

    location_relation = relationship("Location", backref="inventories")
    item_relation = relationship("Item", backref="inventories")


class Orders(Base):
    __tablename__ = "Orders"
    order_id = Column(Integer, primary_key=True, autoincrement=True)
    customer_id = Column(Integer, nullable=False)  # 외래키 제약조건은 고객 테이블이 있으면 추가 가능
    order_dttm = Column(DateTime, nullable=False)
    order_status = Column(Integer, nullable=False)  # 0~3 상태 값
    destination = Column(String(40), nullable=True)  # NULL 허용


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

    order = relationship("Orders", backref="order_items")
    item = relationship("Item", backref="order_items")


class Outbound(Base):
    __tablename__ = "Outbound"
    ob_dttm = Column(DateTime, primary_key=True, autoincrement=True)  # 출고일자 + 시각 (PK)
    order_id = Column(Integer, ForeignKey("Orders.order_id"), nullable=False)
    ob_id = Column(Integer, nullable=False)  # 출고시 바코드 번호
    ob_status = Column(Integer, nullable=False)  # 0/1 출고상태
    destination = Column(String(40), nullable=True)

    order = relationship("Orders", backref="outbounds")


class Inbound(Base):
    __tablename__ = "Inbound"
    inbound_id = Column(Integer, primary_key=True, autoincrement=True)
    item_id = Column(Integer, ForeignKey("Item.item_id"), nullable=False)
    item_amount = Column(Integer, nullable=False)  # 한 박스 별 상품 개수
    ib_amount = Column(Integer, nullable=False)  # 입고된 박스 수량
    ib_status = Column(Integer, nullable=False)  # 입고 진행 상황

    item = relationship("Item", backref="inbounds")

    __table_args__ = (
        CheckConstraint('item_amount > 0', name='check_item_amount_positive'),
        CheckConstraint('ib_amount >= 0', name='check_ib_amount_nonnegative'),
    )
