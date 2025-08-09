#!/usr/bin/env python3
from enum import Enum
from queue import PriorityQueue
import time
import threading
from typing import Dict, List, Optional
import json
from dataclasses import dataclass, asdict
from datetime import datetime
import sqlite3


class TaskStatus(Enum):
    PENDING = "Pending"
    ASSIGNED = "Assigned"
    IN_PROGRESS = "In Progress"
    WAITING_USER = "Waiting User"
    USER_CONFIRMED = "User Confirmed"
    COMPLETED = "Completed"
    FAILED = "Failed"
    CANCELLED = "Cancelled"


class TaskType(Enum):
    MOVE = "move"
    WAIT_USER = "wait_user"
    LOAD = "load"  # 로봇팔이 팔레트에서 모바일로봇으로 적재
    UNLOAD = "unload"  # 로봇팔이 모바일로봇에서 팔레트로 하역
    USER_CHECK = "user_check"  # 사용자 확인 필요
    CALL_WORKER = "call_worker"  # 작업자 호출
    PICK_ITEMS = "pick_items"  # 집품 작업


class RobotType(Enum):
    MOBILE = "mobile"
    ARM = "robot_arm"


class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"
    CHARGING = "Charge"
    LOWBATTERY = "Lowbattery"
    MAINTENANCE = "Maintenance"


class ProcessType(Enum):
    INBOUND = "inbound"  # 입고 프로세스
    OUTBOUND = "outbound"  # 출고 프로세스


class WorkerStatus(Enum):
    AVAILABLE = "Available"
    BUSY = "Busy"
    BREAK = "Break"


@dataclass
class Item:
    """물품 정보"""
    item_id: str
    name: str
    quantity: int
    location: tuple = None
    barcode: str = None


@dataclass
class Order:
    """주문 정보"""
    order_id: str
    items: List[Item]
    priority: int = 1
    order_time: datetime = None
    due_time: datetime = None
    status: str = "pending"


class Task:
    def __init__(self, task_id, task_type: TaskType, robot_type: RobotType, location, 
                 priority=1, items: List[Item] = None, estimated_time=60):
        self.task_id = task_id
        self.task_type = task_type
        self.robot_type = robot_type
        self.location = location
        self.priority = priority
        self.status = TaskStatus.PENDING
        self.assigned_robot = None
        self.items = items or []
        self.estimated_time = estimated_time
        self.created_time = datetime.now()
        self.start_time = None
        self.end_time = None
        self.worker_id = None

    def __lt__(self, other):
        return self.priority < other.priority

    def start_execution(self):
        self.status = TaskStatus.IN_PROGRESS
        self.start_time = datetime.now()

    def complete(self):
        self.status = TaskStatus.COMPLETED
        self.end_time = datetime.now()

    def get_execution_time(self):
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0


class ProcessTask:
    """복합 작업 프로세스"""
    def __init__(self, process_id, process_type: ProcessType, steps: List[Task], 
                 items: List[Item] = None, priority=1):
        self.process_id = process_id
        self.process_type = process_type
        self.steps = steps
        self.current_index = 0
        self.assigned_mobile_robot_id = None
        self.items = items or []
        self.priority = priority
        self.created_time = datetime.now()
        self.start_time = None
        self.end_time = None
        self.total_items = sum(item.quantity for item in self.items)

    def current_step(self):
        if self.current_index < len(self.steps):
            return self.steps[self.current_index]
        return None

    def advance(self):
        if self.current_index < len(self.steps):
            current = self.steps[self.current_index]
            current.complete()
        self.current_index += 1
        
        if self.current_index < len(self.steps):
            next_step = self.steps[self.current_index]
            if self.start_time is None:
                self.start_time = datetime.now()
        else:
            self.end_time = datetime.now()

    def is_done(self):
        return self.current_index >= len(self.steps)
    
    def is_mobile_robot_locked(self, robot_id):
        return self.assigned_mobile_robot_id == robot_id and not self.is_done()

    def get_progress_percent(self):
        return (self.current_index / len(self.steps)) * 100 if self.steps else 100


class Robot:
    def __init__(self, robot_id, robot_type: RobotType, position=(0, 0), 
                 max_load_capacity=10, battery_level=100):
        self.id = robot_id
        self.robot_type = robot_type
        self.status = RobotStatus.IDLE
        self.position = position
        self.current_task: Task = None
        self.max_load_capacity = max_load_capacity
        self.current_load = 0
        self.battery_level = battery_level
        self.last_maintenance = datetime.now()
        self.total_tasks_completed = 0

    def is_available(self):
        return (self.status == RobotStatus.IDLE and 
                self.battery_level > 20 and 
                self.current_load < self.max_load_capacity)

    def assign_task(self, task: Task):
        self.current_task = task
        self.status = RobotStatus.BUSY
        task.start_execution()

    def complete_task(self):
        if self.current_task:
            self.current_task.complete()
            self.total_tasks_completed += 1
            self.current_task = None
            self.status = RobotStatus.IDLE
            # 배터리 소모 시뮬레이션
            self.battery_level -= 5
            if self.battery_level <= 20:
                self.status = RobotStatus.LOWBATTERY


class Worker:
    def __init__(self, worker_id, name, skills: List[str] = None):
        self.id = worker_id
        self.name = name
        self.status = WorkerStatus.AVAILABLE
        self.skills = skills or []
        self.current_location = None
        self.assigned_tasks = []

    def is_available(self):
        return self.status == WorkerStatus.AVAILABLE

    def assign_task(self, task: Task):
        self.assigned_tasks.append(task)
        self.status = WorkerStatus.BUSY
        task.worker_id = self.id


class DatabaseManager:
    """데이터베이스 관리"""
    def __init__(self, db_path="warehouse.db"):
        self.db_path = db_path
        self.init_database()

    def init_database(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # 주문 테이블
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS orders (
                order_id TEXT PRIMARY KEY,
                items TEXT,
                priority INTEGER,
                order_time TEXT,
                due_time TEXT,
                status TEXT
            )
        ''')
        
        # 재고 테이블
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS inventory (
                item_id TEXT PRIMARY KEY,
                name TEXT,
                quantity INTEGER,
                location TEXT
            )
        ''')
        
        conn.commit()
        conn.close()

    def add_order(self, order: Order):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        items_json = json.dumps([asdict(item) for item in order.items])
        cursor.execute('''
            INSERT OR REPLACE INTO orders 
            (order_id, items, priority, order_time, due_time, status)
            VALUES (?, ?, ?, ?, ?, ?)
        ''', (order.order_id, items_json, order.priority, 
              order.order_time.isoformat() if order.order_time else None,
              order.due_time.isoformat() if order.due_time else None,
              order.status))
        
        conn.commit()
        conn.close()

    def get_pending_orders(self) -> List[Order]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('SELECT * FROM orders WHERE status = "pending"')
        orders = []
        
        for row in cursor.fetchall():
            items_data = json.loads(row[1])
            items = [Item(**item_data) for item_data in items_data]
            
            order = Order(
                order_id=row[0],
                items=items,
                priority=row[2],
                order_time=datetime.fromisoformat(row[3]) if row[3] else None,
                due_time=datetime.fromisoformat(row[4]) if row[4] else None,
                status=row[5]
            )
            orders.append(order)
        
        conn.close()
        return orders


class TaskManager:
    def __init__(self):
        self.task_queue = PriorityQueue()
        self.all_process_tasks: List[ProcessTask] = []
        self.completed_processes: List[ProcessTask] = []
        self.robots = {
            1: Robot(1, RobotType.MOBILE, (0, 0)),
            2: Robot(2, RobotType.MOBILE, (10, 10)),
            3: Robot(3, RobotType.MOBILE, (20, 10)),
            4: Robot(4, RobotType.ARM, (3, 10)),
            5: Robot(5, RobotType.ARM, (3, 2)),
            6: Robot(6, RobotType.ARM, (15, 5)),
        }
        self.workers = {
            1: Worker(1, "김작업자", ["picking", "checking"]),
            2: Worker(2, "이작업자", ["picking", "loading"]),
            3: Worker(3, "박작업자", ["checking", "quality_control"]),
        }
        self.db_manager = DatabaseManager()
        self.order_monitor_active = False
        self.statistics = {
            'total_processes': 0,
            'completed_processes': 0,
            'active_robots': 0,
            'average_completion_time': 0
        }

    def add_process_task(self, process: ProcessTask):
        """프로세스 작업 추가"""
        self.all_process_tasks.append(process)
        self.statistics['total_processes'] += 1
        
        # 첫 번째 단계를 큐에 추가
        step = process.current_step()
        if step:
            self.task_queue.put((step.priority, time.time(), step))
        
        print(f"🔄 프로세스 {process.process_id} 추가됨 (총 {len(process.steps)}단계)")

    def start_order_monitoring(self):
        """주문 모니터링 시작"""
        if not self.order_monitor_active:
            self.order_monitor_active = True
            monitoring_thread = threading.Thread(target=self._monitor_orders)
            monitoring_thread.daemon = True
            monitoring_thread.start()
            print("📊 주문 모니터링 시작됨")

    def stop_order_monitoring(self):
        """주문 모니터링 중지"""
        self.order_monitor_active = False
        print("📊 주문 모니터링 중지됨")

    def _monitor_orders(self):
        """주기적으로 주문 DB 확인"""
        while self.order_monitor_active:
            try:
                pending_orders = self.db_manager.get_pending_orders()
                for order in pending_orders:
                    # 출고 프로세스 자동 생성
                    outbound_process = self.create_outbound_process_from_order(order)
                    self.add_process_task(outbound_process)
                    
                    # 주문 상태 업데이트
                    order.status = "processing"
                    self.db_manager.add_order(order)
                
                time.sleep(30)  # 30초마다 확인
            except Exception as e:
                print(f"❌ 주문 모니터링 오류: {e}")
                time.sleep(60)

    def create_outbound_process_from_order(self, order: Order) -> ProcessTask:
        """주문으로부터 출고 프로세스 생성"""
        steps = []
        
        for i, item in enumerate(order.items):
            item_location = item.location or (5, 5)  # 기본 위치
            
            steps.extend([
                Task(f"{order.order_id}_move_{i}", TaskType.MOVE, RobotType.MOBILE, 
                     item_location, order.priority),
                Task(f"{order.order_id}_call_{i}", TaskType.CALL_WORKER, RobotType.MOBILE, 
                     item_location, order.priority),
                Task(f"{order.order_id}_pick_{i}", TaskType.PICK_ITEMS, RobotType.MOBILE, 
                     item_location, order.priority, [item]),
                Task(f"{order.order_id}_check_{i}", TaskType.USER_CHECK, RobotType.MOBILE, 
                     item_location, order.priority, [item])
            ])
        
        # 최종 출고장으로 이동 및 하역
        steps.extend([
            Task(f"{order.order_id}_final_move", TaskType.MOVE, RobotType.MOBILE, 
                 (25, 25), order.priority),  # 출고장 위치
            Task(f"{order.order_id}_unload", TaskType.UNLOAD, RobotType.ARM, 
                 (25, 25), order.priority, order.items)
        ])
        
        return ProcessTask(f"OUT_{order.order_id}", ProcessType.OUTBOUND, 
                          steps, order.items, order.priority)

    def assign_tasks(self):
        """작업 할당"""
        messages = []
        new_queue = PriorityQueue()
        assigned_count = 0

        while not self.task_queue.empty():
            _, timestamp, task = self.task_queue.get()

            # 프로세스 찾기
            process = next((p for p in self.all_process_tasks if task in p.steps), None)
            
            # 로봇 선택
            robot = self._select_optimal_robot(task, process)
            
            # 작업자가 필요한 작업 처리
            if task.task_type in [TaskType.USER_CHECK, TaskType.CALL_WORKER, TaskType.PICK_ITEMS]:
                worker = self._select_worker(task)
                if not worker:
                    messages.append(f"⏸ 작업자가 없어서 Task {task.task_id} 보류")
                    new_queue.put((task.priority, timestamp, task))
                    continue
                worker.assign_task(task)

            if robot:
                task.status = TaskStatus.ASSIGNED
                robot.assign_task(task)
                
                # 모바일 로봇 할당 기억
                if task.robot_type == RobotType.MOBILE and process and not process.assigned_mobile_robot_id:
                    process.assigned_mobile_robot_id = robot.id

                messages.append(f"✅ Task {task.task_id} [{task.task_type.name}] → Robot {robot.id}")
                assigned_count += 1
            else:
                messages.append(f"⏸ Robot unavailable for Task {task.task_id}")
                new_queue.put((task.priority, timestamp, task))

        self.task_queue = new_queue
        self.statistics['active_robots'] = sum(1 for r in self.robots.values() if r.status == RobotStatus.BUSY)
        
        return f"🚚 {assigned_count}개 작업 할당 완료\n" + "\n".join(messages)

    def _select_optimal_robot(self, task: Task, process: ProcessTask = None):
        """최적 로봇 선택"""
        available_robots = [r for r in self.robots.values() 
                           if r.is_available() and r.robot_type == task.robot_type]
        
        if not available_robots:
            return None

        # 모바일 로봇의 경우 프로세스 고정 확인
        if task.robot_type == RobotType.MOBILE and process:
            if process.assigned_mobile_robot_id:
                fixed_robot = self.robots.get(process.assigned_mobile_robot_id)
                if fixed_robot and fixed_robot.is_available():
                    return fixed_robot
            
            # 다른 프로세스에 고정된 로봇 제외
            available_robots = [r for r in available_robots 
                              if not any(p.assigned_mobile_robot_id == r.id and not p.is_done() 
                                       for p in self.all_process_tasks)]

        if not available_robots:
            return None

        # 거리 기반 최적 로봇 선택 (모바일 로봇)
        if task.robot_type == RobotType.MOBILE:
            return min(available_robots, 
                      key=lambda r: ((r.position[0] - task.location[0])**2 + 
                                   (r.position[1] - task.location[1])**2))
        
        # 로봇팔의 경우 용량과 거리 고려
        return min(available_robots,
                  key=lambda r: (r.current_load, 
                               ((r.position[0] - task.location[0])**2 + 
                                (r.position[1] - task.location[1])**2)))

    def _select_worker(self, task: Task):
        """작업자 선택"""
        skill_required = {
            TaskType.USER_CHECK: "checking",
            TaskType.PICK_ITEMS: "picking",
            TaskType.CALL_WORKER: "general"
        }.get(task.task_type, "general")
        
        available_workers = [w for w in self.workers.values() 
                           if w.is_available() and 
                           (skill_required in w.skills or skill_required == "general")]
        
        return available_workers[0] if available_workers else None

    def complete_task(self, robot_id, success=True):
        """작업 완료 처리"""
        robot = self.robots.get(robot_id)
        if not robot or not robot.current_task:
            return f"ℹ️ Robot {robot_id}에 활성 작업이 없습니다."

        task = robot.current_task
        process = next((p for p in self.all_process_tasks if task in p.steps), None)
        
        if success:
            robot.complete_task()
            
            # 작업자가 할당된 경우 해제
            if task.worker_id:
                worker = self.workers.get(task.worker_id)
                if worker:
                    worker.status = WorkerStatus.AVAILABLE
                    if task in worker.assigned_tasks:
                        worker.assigned_tasks.remove(task)

            if process:
                process.advance()
                next_step = process.current_step()
                
                if next_step:
                    # 다음 단계를 큐에 추가
                    self.task_queue.put((next_step.priority, time.time(), next_step))
                    return f"✅ {task.task_id} 완료, 다음: {next_step.task_type.name}"
                else:
                    # 프로세스 완료
                    self.completed_processes.append(process)
                    self.statistics['completed_processes'] += 1
                    self._update_completion_stats()
                    return f"🎉 프로세스 {process.process_id} 완전히 완료! ({len(process.steps)}단계)"
        else:
            task.status = TaskStatus.FAILED
            robot.status = RobotStatus.IDLE
            robot.current_task = None
            return f"❌ Task {task.task_id} 실패 처리됨"

        return f"✅ Task {task.task_id} 완료"

    def _update_completion_stats(self):
        """완료 통계 업데이트"""
        if self.completed_processes:
            total_time = sum((p.end_time - p.start_time).total_seconds() 
                           for p in self.completed_processes 
                           if p.start_time and p.end_time)
            self.statistics['average_completion_time'] = total_time / len(self.completed_processes)

    def get_system_status(self):
        """시스템 상태 조회"""
        active_processes = len(self.all_process_tasks) - len(self.completed_processes)
        
        robot_status = {}
        for robot_type in RobotType:
            robots_of_type = [r for r in self.robots.values() if r.robot_type == robot_type]
            robot_status[robot_type.name] = {
                'total': len(robots_of_type),
                'idle': sum(1 for r in robots_of_type if r.status == RobotStatus.IDLE),
                'busy': sum(1 for r in robots_of_type if r.status == RobotStatus.BUSY),
                'low_battery': sum(1 for r in robots_of_type if r.status == RobotStatus.LOWBATTERY)
            }
        
        return {
            'active_processes': active_processes,
            'completed_processes': len(self.completed_processes),
            'pending_tasks': self.task_queue.qsize(),
            'robot_status': robot_status,
            'worker_status': {
                'available': sum(1 for w in self.workers.values() if w.is_available()),
                'busy': sum(1 for w in self.workers.values() if w.status == WorkerStatus.BUSY)
            },
            'statistics': self.statistics
        }

    def simulate_user_confirmation(self, process_id, confirmed=True):
        """사용자 확인 시뮬레이션"""
        process = next((p for p in self.all_process_tasks if p.process_id == process_id), None)
        if not process:
            return f"❌ 프로세스 {process_id}를 찾을 수 없습니다."
        
        current_task = process.current_step()
        if current_task and current_task.task_type == TaskType.USER_CHECK:
            if confirmed:
                current_task.status = TaskStatus.USER_CONFIRMED
                return f"✅ 프로세스 {process_id}의 사용자 확인 완료"
            else:
                current_task.status = TaskStatus.FAILED
                return f"❌ 프로세스 {process_id}의 사용자 확인 실패"
        
        return f"ℹ️ 프로세스 {process_id}는 현재 사용자 확인이 필요하지 않습니다."


def create_inbound_process(process_id, items: List[Item], start_pos=(1, 1), 
                          display_positions=None) -> ProcessTask:
    """입고 프로세스 생성"""
    if display_positions is None:
        display_positions = [(5, 5), (6, 5), (7, 5)]
    
    steps = []
    
    # 각 물품 그룹별로 작업 생성
    for i, item in enumerate(items):
        display_pos = display_positions[i % len(display_positions)]
        
        steps.extend([
            Task(f"{process_id}_move_{i}", TaskType.MOVE, RobotType.MOBILE, start_pos, 
                 priority=1, items=[item]),
            Task(f"{process_id}_load_{i}", TaskType.LOAD, RobotType.ARM, start_pos, 
                 priority=1, items=[item]),
            Task(f"{process_id}_transport_{i}", TaskType.MOVE, RobotType.MOBILE, display_pos, 
                 priority=1, items=[item]),
            Task(f"{process_id}_check_{i}", TaskType.USER_CHECK, RobotType.MOBILE, display_pos, 
                 priority=1, items=[item])
        ])
    
    return ProcessTask(process_id, ProcessType.INBOUND, steps, items)


def create_outbound_process(process_id, items: List[Item], pick_positions=None, 
                           drop_pos=(25, 25)) -> ProcessTask:
    """출고 프로세스 생성"""
    if pick_positions is None:
        pick_positions = [(5, 5), (6, 5), (7, 5)]
    
    steps = []
    
    # 각 물품 집품
    for i, item in enumerate(items):
        pick_pos = pick_positions[i % len(pick_positions)]
        
        steps.extend([
            Task(f"{process_id}_move_{i}", TaskType.MOVE, RobotType.MOBILE, pick_pos, 
                 priority=1, items=[item]),
            Task(f"{process_id}_call_{i}", TaskType.CALL_WORKER, RobotType.MOBILE, pick_pos, 
                 priority=1, items=[item]),
            Task(f"{process_id}_pick_{i}", TaskType.PICK_ITEMS, RobotType.MOBILE, pick_pos, 
                 priority=1, items=[item]),
            Task(f"{process_id}_check_{i}", TaskType.USER_CHECK, RobotType.MOBILE, pick_pos, 
                 priority=1, items=[item])
        ])
    
    # 최종 출고
    steps.extend([
        Task(f"{process_id}_final_move", TaskType.MOVE, RobotType.MOBILE, drop_pos, 
             priority=1, items=items),
        Task(f"{process_id}_unload", TaskType.UNLOAD, RobotType.ARM, drop_pos, 
             priority=1, items=items)
    ])
    
    return ProcessTask(process_id, ProcessType.OUTBOUND, steps, items)


# GUI 부분은 PySide6 사용 (기존 코드 개선)
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel,
    QTableWidget, QTableWidgetItem, QHBoxLayout, QTabWidget, QTextEdit,
    QSpinBox, QGroupBox, QGridLayout, QProgressBar, QComboBox, QLineEdit
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
import sys


class WarehouseManagementUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.counter = 1
        self.manager = TaskManager()
        self.setup_ui()
        self.setup_timer()
        
        # 주문 모니터링 시작
        self.manager.start_order_monitoring()

    def setup_ui(self):
        self.setWindowTitle("🏭 자동화 물류센터 관제 시스템")
        self.setGeometry(100, 100, 1200, 800)
        
        # 중앙 위젯과 탭
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # 제목
        title = QLabel("🏭 자동화 물류센터 관제 시스템")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        layout.addWidget(title)
        
        # 탭 위젯
        tab_widget = QTabWidget()
        layout.addWidget(tab_widget)
        
        # 각 탭 생성
        tab_widget.addTab(self.create_control_tab(), "🎮 작업 제어")
        tab_widget.addTab(self.create_status_tab(), "📊 시스템 상태")
        tab_widget.addTab(self.create_robot_tab(), "🤖 로봇 관리")
        tab_widget.addTab(self.create_order_tab(), "📦 주문 관리")

    def create_control_tab(self):
        """작업 제어 탭"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 상단 컨트롤
        control_group = QGroupBox("작업 제어")
        control_layout = QHBoxLayout(control_group)
        
        btn_inbound = QPushButton("📥 입고 작업 추가")
        btn_inbound.clicked.connect(self.add_inbound_process)
        control_layout.addWidget(btn_inbound)
        
        btn_outbound = QPushButton("📤 출고 작업 추가") 
        btn_outbound.clicked.connect(self.add_outbound_process)
        control_layout.addWidget(btn_outbound)
        
        btn_assign = QPushButton("🚚 작업 할당")
        btn_assign.clicked.connect(self.assign_tasks)
        control_layout.addWidget(btn_assign)
        
        btn_emergency = QPushButton("🚨 비상 정지")
        btn_emergency.setStyleSheet("background-color: red; color: white;")
        btn_emergency.clicked.connect(self.emergency_stop)
        control_layout.addWidget(btn_emergency)
        
        layout.addWidget(control_group)
        
        # 로봇 완료 버튼들
        robot_group = QGroupBox("로봇 작업 완료")
        robot_layout = QGridLayout(robot_group)
        
        for i, (robot_id, robot) in enumerate(self.manager.robots.items()):
            btn = QPushButton(f"✅ {robot.robot_type.name} {robot_id}")
            btn.clicked.connect(lambda _, rid=robot_id: self.complete_task(rid))
            robot_layout.addWidget(btn, i // 3, i % 3)
        
        layout.addWidget(robot_group)
        
        # 프로세스 테이블
        self.process_table = QTableWidget()
        self.process_table.setColumnCount(8)
        self.process_table.setHorizontalHeaderLabels([
            "프로세스 ID", "유형", "진행률", "현재 단계", "할당 로봇", "물품 수", "생성 시간", "상태"
        ])
        layout.addWidget(self.process_table)
        
        # 로그 영역
        log_group = QGroupBox("시스템 로그")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)
        
        return widget

    def create_status_tab(self):
        """시스템 상태 탭"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 통계 정보
        stats_group = QGroupBox("시스템 통계")
        stats_layout = QGridLayout(stats_group)
        
        self.stats_labels = {}
        stats_info = [
            ("총 프로세스", "total_processes"),
            ("완료된 프로세스", "completed_processes"), 
            ("활성 로봇", "active_robots"),
            ("대기 작업", "pending_tasks"),
            ("평균 완료시간", "avg_time"),
            ("시스템 효율", "efficiency")
        ]
        
        for i, (label, key) in enumerate(stats_info):
            lbl = QLabel(f"{label}:")
            value_lbl = QLabel("0")
            value_lbl.setStyleSheet("font-weight: bold; color: blue;")
            self.stats_labels[key] = value_lbl
            
            stats_layout.addWidget(lbl, i // 2, (i % 2) * 2)
            stats_layout.addWidget(value_lbl, i // 2, (i % 2) * 2 + 1)
        
        layout.addWidget(stats_group)
        
        # 작업 진행률 시각화
        progress_group = QGroupBox("진행 중인 프로세스")
        progress_layout = QVBoxLayout(progress_group)
        self.progress_bars = {}
        progress_layout.addWidget(QLabel("진행률이 여기에 표시됩니다"))
        layout.addWidget(progress_group)
        
        return widget

    def create_robot_tab(self):
        """로봇 관리 탭"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 로봇 상태 테이블
        self.robot_table = QTableWidget()
        self.robot_table.setColumnCount(7)
        self.robot_table.setHorizontalHeaderLabels([
            "로봇 ID", "유형", "상태", "위치", "배터리", "현재 작업", "완료된 작업 수"
        ])
        layout.addWidget(self.robot_table)
        
        # 로봇 제어 버튼들
        control_group = QGroupBox("로봇 제어")
        control_layout = QHBoxLayout(control_group)
        
        btn_charge_all = QPushButton("🔋 모든 로봇 충전")
        btn_charge_all.clicked.connect(self.charge_all_robots)
        control_layout.addWidget(btn_charge_all)
        
        btn_maintenance = QPushButton("🔧 정비 모드")
        btn_maintenance.clicked.connect(self.toggle_maintenance_mode)
        control_layout.addWidget(btn_maintenance)
        
        layout.addWidget(control_group)
        
        return widget

    def create_order_tab(self):
        """주문 관리 탭"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 주문 생성
        order_group = QGroupBox("주문 생성 (시뮬레이션)")
        order_layout = QGridLayout(order_group)
        
        order_layout.addWidget(QLabel("주문 ID:"), 0, 0)
        self.order_id_input = QLineEdit(f"ORDER_{self.counter:03d}")
        order_layout.addWidget(self.order_id_input, 0, 1)
        
        order_layout.addWidget(QLabel("물품명:"), 1, 0)
        self.item_name_input = QLineEdit("테스트물품")
        order_layout.addWidget(self.item_name_input, 1, 1)
        
        order_layout.addWidget(QLabel("수량:"), 2, 0)
        self.quantity_input = QSpinBox()
        self.quantity_input.setRange(1, 100)
        self.quantity_input.setValue(5)
        order_layout.addWidget(self.quantity_input, 2, 1)
        
        order_layout.addWidget(QLabel("우선순위:"), 3, 0)
        self.priority_input = QSpinBox()
        self.priority_input.setRange(1, 10)
        self.priority_input.setValue(1)
        order_layout.addWidget(self.priority_input, 3, 1)
        
        btn_create_order = QPushButton("📦 주문 생성")
        btn_create_order.clicked.connect(self.create_test_order)
        order_layout.addWidget(btn_create_order, 4, 0, 1, 2)
        
        layout.addWidget(order_group)
        
        # 주문 목록
        order_list_group = QGroupBox("주문 목록")
        order_list_layout = QVBoxLayout(order_list_group)
        
        self.order_table = QTableWidget()
        self.order_table.setColumnCount(6)
        self.order_table.setHorizontalHeaderLabels([
            "주문 ID", "물품", "수량", "우선순위", "주문시간", "상태"
        ])
        order_list_layout.addWidget(self.order_table)
        
        layout.addWidget(order_list_group)
        
        return widget

    def setup_timer(self):
        """타이머 설정"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(2000)  # 2초마다 업데이트

    def add_inbound_process(self):
        """입고 프로세스 추가"""
        items = [
            Item(f"ITEM_{self.counter:03d}_1", "입고물품A", 10, (1, 1)),
            Item(f"ITEM_{self.counter:03d}_2", "입고물품B", 5, (1, 1))
        ]
        
        process = create_inbound_process(f"IN_{self.counter:03d}", items)
        self.manager.add_process_task(process)
        
        self.log_message(f"📥 입고 프로세스 {process.process_id} 추가됨 ({len(items)}개 물품)")
        self.counter += 1

    def add_outbound_process(self):
        """출고 프로세스 추가"""
        items = [
            Item(f"ITEM_OUT_{self.counter:03d}_1", "출고물품A", 3, (5, 5)),
            Item(f"ITEM_OUT_{self.counter:03d}_2", "출고물품B", 2, (6, 5))
        ]
        
        process = create_outbound_process(f"OUT_{self.counter:03d}", items)
        self.manager.add_process_task(process)
        
        self.log_message(f"📤 출고 프로세스 {process.process_id} 추가됨 ({len(items)}개 물품)")
        self.counter += 1

    def assign_tasks(self):
        """작업 할당"""
        result = self.manager.assign_tasks()
        self.log_message(result)

    def complete_task(self, robot_id):
        """작업 완료 처리"""
        result = self.manager.complete_task(robot_id)
        self.log_message(result)

    def emergency_stop(self):
        """비상 정지"""
        # 모든 로봇을 IDLE 상태로
        for robot in self.manager.robots.values():
            if robot.current_task:
                robot.current_task.status = TaskStatus.CANCELLED
            robot.status = RobotStatus.IDLE
            robot.current_task = None
        
        self.log_message("🚨 비상 정지 실행됨 - 모든 로봇 작업 중단")

    def charge_all_robots(self):
        """모든 로봇 충전"""
        charged_count = 0
        for robot in self.manager.robots.values():
            if robot.status != RobotStatus.BUSY:
                robot.battery_level = 100
                robot.status = RobotStatus.IDLE
                charged_count += 1
        
        self.log_message(f"🔋 {charged_count}개 로봇 충전 완료")

    def toggle_maintenance_mode(self):
        """정비 모드 토글"""
        maintenance_count = sum(1 for r in self.manager.robots.values() 
                              if r.status == RobotStatus.MAINTENANCE)
        
        if maintenance_count == 0:
            # 정비 모드 시작
            for robot in self.manager.robots.values():
                if robot.status == RobotStatus.IDLE:
                    robot.status = RobotStatus.MAINTENANCE
            self.log_message("🔧 정비 모드 시작")
        else:
            # 정비 모드 종료
            for robot in self.manager.robots.values():
                if robot.status == RobotStatus.MAINTENANCE:
                    robot.status = RobotStatus.IDLE
            self.log_message("🔧 정비 모드 종료")

    def create_test_order(self):
        """테스트 주문 생성"""
        order_id = self.order_id_input.text()
        item_name = self.item_name_input.text()
        quantity = self.quantity_input.value()
        priority = self.priority_input.value()
        
        items = [Item(f"{order_id}_ITEM", item_name, quantity, (5, 5))]
        order = Order(order_id, items, priority, datetime.now(), 
                     datetime.now(), "pending")
        
        self.manager.db_manager.add_order(order)
        self.log_message(f"📦 테스트 주문 {order_id} 생성됨")
        
        # 입력 필드 초기화
        self.counter += 1
        self.order_id_input.setText(f"ORDER_{self.counter:03d}")

    def update_ui(self):
        """UI 업데이트"""
        self.update_process_table()
        self.update_robot_table()
        self.update_statistics()
        self.update_order_table()

    def update_process_table(self):
        """프로세스 테이블 업데이트"""
        processes = self.manager.all_process_tasks
        self.process_table.setRowCount(len(processes))
        
        for row, process in enumerate(processes):
            current_step = process.current_step()
            assigned_robot = ""
            
            if current_step and current_step.assigned_robot:
                robot = self.manager.robots.get(current_step.assigned_robot)
                assigned_robot = f"{robot.robot_type.name} {robot.id}" if robot else ""
            
            items = [
                process.process_id,
                process.process_type.value,
                f"{process.get_progress_percent():.1f}%",
                current_step.task_type.name if current_step else "완료",
                assigned_robot,
                str(process.total_items),
                process.created_time.strftime("%H:%M:%S"),
                "완료" if process.is_done() else "진행중"
            ]
            
            for col, item in enumerate(items):
                self.process_table.setItem(row, col, QTableWidgetItem(str(item)))

    def update_robot_table(self):
        """로봇 테이블 업데이트"""
        robots = list(self.manager.robots.values())
        self.robot_table.setRowCount(len(robots))
        
        for row, robot in enumerate(robots):
            current_task = robot.current_task.task_id if robot.current_task else "-"
            
            items = [
                str(robot.id),
                robot.robot_type.value,
                robot.status.value,
                f"({robot.position[0]}, {robot.position[1]})",
                f"{robot.battery_level}%",
                current_task,
                str(robot.total_tasks_completed)
            ]
            
            for col, item in enumerate(items):
                widget_item = QTableWidgetItem(str(item))
                
                # 상태에 따른 색상 설정
                if col == 2:  # 상태 컬럼
                    if robot.status == RobotStatus.BUSY:
                        widget_item.setBackground(Qt.GlobalColor.yellow)
                    elif robot.status == RobotStatus.LOWBATTERY:
                        widget_item.setBackground(Qt.GlobalColor.red)
                    elif robot.status == RobotStatus.IDLE:
                        widget_item.setBackground(Qt.GlobalColor.green)
                
                self.robot_table.setItem(row, col, widget_item)

    def update_statistics(self):
        """통계 정보 업데이트"""
        status = self.manager.get_system_status()
        
        self.stats_labels["total_processes"].setText(str(status['statistics']['total_processes']))
        self.stats_labels["completed_processes"].setText(str(status['completed_processes']))
        self.stats_labels["active_robots"].setText(str(status['active_processes']))
        self.stats_labels["pending_tasks"].setText(str(status['pending_tasks']))
        
        avg_time = status['statistics']['average_completion_time']
        self.stats_labels["avg_time"].setText(f"{avg_time:.1f}초")
        
        # 효율 계산 (완료율)
        total = status['statistics']['total_processes']
        completed = status['completed_processes']
        efficiency = (completed / total * 100) if total > 0 else 0
        self.stats_labels["efficiency"].setText(f"{efficiency:.1f}%")

    def update_order_table(self):
        """주문 테이블 업데이트"""
        try:
            orders = self.manager.db_manager.get_pending_orders()
            self.order_table.setRowCount(len(orders))
            
            for row, order in enumerate(orders):
                items_str = ", ".join([f"{item.name}({item.quantity})" for item in order.items])
                order_time = order.order_time.strftime("%H:%M:%S") if order.order_time else "-"
                
                items = [
                    order.order_id,
                    items_str,
                    str(sum(item.quantity for item in order.items)),
                    str(order.priority),
                    order_time,
                    order.status
                ]
                
                for col, item in enumerate(items):
                    self.order_table.setItem(row, col, QTableWidgetItem(str(item)))
        except Exception as e:
            self.log_message(f"❌ 주문 테이블 업데이트 오류: {e}")

    def log_message(self, message):
        """로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.log_text.append(formatted_message)
        
        # 로그가 너무 길면 줄 수 제한
        if self.log_text.document().blockCount() > 100:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.BlockUnderCursor)
            cursor.removeSelectedText()

    def closeEvent(self, event):
        """애플리케이션 종료 시 정리"""
        self.manager.stop_order_monitoring()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 스타일 설정
    app.setStyle('Fusion')
    
    window = WarehouseManagementUI()
    window.show()
    
    sys.exit(app.exec())