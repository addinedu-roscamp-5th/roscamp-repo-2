from enum import Enum
from queue import PriorityQueue
import time


class TaskStatus(Enum):
    PENDING = "Pending"
    ASSIGNED = "Assigned"
    IN_PROGRESS = "In Progress"
    COMPLETED = "Completed"
    FAILED = "Failed"
    CANCELLED = "Cancelled"


class TaskType(Enum):
    MOVE = "move"
    WAIT_USER = "wait_user"
    LOAD = "load : pick & place on boltbot"
    UNLOAD = "unload : pick & place from boltbot"


class RobotType(Enum):
    MOBILE = "mobile"
    ARM = "robot_arm"


class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"
    CHARGING = "Charge"
    LOWBATTERY = "Lowbattery"


class Task:
    def __init__(self, task_id, task_type: TaskType, robot_type: RobotType, location, priority=1):
        self.task_id = task_id
        self.task_type = task_type
        self.robot_type = robot_type
        self.location = location
        self.priority = priority
        self.status = TaskStatus.PENDING
        self.assigned_robot = None

    def __lt__(self, other):
        return self.priority < other.priority


class CompositeTask:
    def __init__(self, task_id, steps: list[Task]):
        self.task_id = task_id
        self.steps = steps
        self.current_index = 0
        self.assigned_mobile_robot_id = None

    def current_step(self):
        if self.current_index < len(self.steps):
            return self.steps[self.current_index]
        return None

    def advance(self):
        self.current_index += 1

    def is_done(self):
        return self.current_index >= len(self.steps)
    
    def is_mobile_robot_locked(self, robot_id):
        return self.assigned_mobile_robot_id == robot_id and not self.is_done()



class Robot:
    def __init__(self, robot_id, robot_type: RobotType, position=(0, 0)):
        self.id = robot_id
        self.robot_type = robot_type
        self.status = RobotStatus.IDLE
        self.position = position
        self.current_task: Task = None

    def is_available(self):
        return self.status == RobotStatus.IDLE


class TaskManager:
    def __init__(self):
        self.task_queue = PriorityQueue()
        self.all_composite_tasks: list[CompositeTask] = []
        self.robots = {
            1: Robot(1, RobotType.MOBILE, (0, 0)),
            2: Robot(2, RobotType.MOBILE, (10, 10)),
            3: Robot(3, RobotType.MOBILE, (20, 10)),
            4: Robot(4, RobotType.ARM, (3, 10)),
            5: Robot(5, RobotType.ARM, (3, 2)),
        }

    def add_composite_task(self, composite: CompositeTask):
        self.all_composite_tasks.append(composite)
        step = composite.current_step()
        if step:
            self.task_queue.put((step.priority, time.time(), step))

    def assign_tasks(self):
        messages = []
        new_queue = PriorityQueue()

        while not self.task_queue.empty():
            _, _, task = self.task_queue.get()

            # ✅ CompositeTask 찾기
            composite = next((c for c in self.all_composite_tasks if task in c.steps), None)

            if composite and task.robot_type == RobotType.MOBILE and composite.assigned_mobile_robot_id:
                # mobile 작업이면서 이미 로봇이 지정된 경우 해당 로봇 강제 사용
                robot = self.robots.get(composite.assigned_mobile_robot_id)
                if robot and robot.is_available():
                    assigned = True
                else:
                    robot = None  # 지정 로봇이 사용불가면 할당 보류
            else:
                robot = self.select_robot(task)

            if robot:
                task.status = TaskStatus.ASSIGNED
                task.assigned_robot = robot.id
                robot.status = RobotStatus.BUSY
                robot.current_task = task

                # ✅ 처음 할당된 mobile 로봇 기억
                if task.robot_type == RobotType.MOBILE and composite and not composite.assigned_mobile_robot_id:
                    composite.assigned_mobile_robot_id = robot.id

                messages.append(f"✅ Task {task.task_id} [{task.task_type.name}] assigned to Robot {robot.id}")
            else:
                messages.append(f"⏸ No robot available for Task {task.task_id} ({task.robot_type.name})")
                new_queue.put((task.priority, time.time(), task))

        self.task_queue = new_queue
        return "\n".join(messages)


    def select_robot(self, task: Task):
        closest = None
        min_dist = float("inf")
        for robot in self.robots.values():
            if robot.is_available() and robot.robot_type == task.robot_type:
                if robot.robot_type == RobotType.MOBILE:
                    # 다른 CompositeTask에 이미 고정된 로봇인지 확인
                    if any(
                        c.assigned_mobile_robot_id == robot.id and not c.is_done()
                        for c in self.all_composite_tasks
                    ):
                        continue  # 🔒 이미 다른 CompositeTask에 묶여 있음
                dist = (robot.position[0] - task.location[0])**2 + (robot.position[1] - task.location[1])**2
                if dist < min_dist:
                    min_dist = dist
                    closest = robot
        return closest

    def complete_task(self, robot_id):
        r = self.robots.get(robot_id)
        if not r or not r.current_task:
            return f"ℹ️ Robot {robot_id} has no active task."

        task = r.current_task
        task.status = TaskStatus.COMPLETED
        r.status = RobotStatus.IDLE
        r.current_task = None

        # find composite task and advance
        for comp in self.all_composite_tasks:
            if task in comp.steps:
                comp.advance()
                next_step = comp.current_step()
                if next_step:
                    self.task_queue.put((next_step.priority, time.time(), next_step))
                    return f"✅ Completed task {task.task_id}, next step {next_step.task_type.name} enqueued"
                else:
                    return f"✅ Composite task {comp.task_id} fully completed"

        return f"✅ Task {task.task_id} completed by Robot {robot_id}"


def create_inbound_task(composite_id, start_pos=(1, 1), display_pos=(5, 5)):
    steps = [
        Task(f"{composite_id}_1", TaskType.MOVE, RobotType.MOBILE, start_pos),
        Task(f"{composite_id}_2", TaskType.LOAD, RobotType.ARM, start_pos),
        Task(f"{composite_id}_3", TaskType.MOVE, RobotType.MOBILE, display_pos),
        Task(f"{composite_id}_4", TaskType.WAIT_USER, RobotType.MOBILE, display_pos)
    ]
    return CompositeTask(composite_id, steps)

def create_outbound_task(composite_id, pick_pos=(4, 4), drop_pos=(10, 10)) -> CompositeTask:
    steps = [
        Task(f"{composite_id}_1", TaskType.MOVE, RobotType.MOBILE, pick_pos, priority=1),
        Task(f"{composite_id}_2", TaskType.WAIT_USER, RobotType.MOBILE, pick_pos, priority=1),
        Task(f"{composite_id}_3", TaskType.MOVE, RobotType.MOBILE, drop_pos, priority=1),
        Task(f"{composite_id}_4", TaskType.UNLOAD, RobotType.ARM, drop_pos, priority=1),
    ]
    return CompositeTask(composite_id, steps)

    

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
    QTableWidget, QTableWidgetItem, QHBoxLayout
)
from PySide6.QtCore import Qt
import sys

# 여기엔 사용자가 제공한 TaskManager 관련 전체 코드가 이미 정의돼 있어야 합니다.
# 아래 예시는 TaskManager, Task, Robot, TaskType 등 정의된 상태에서만 작동합니다.

class TaskManagerUI(QWidget):
    def __init__(self, manager: TaskManager):
        super().__init__()
        self.manager = manager
        self.setWindowTitle("📦 복합 작업 Task Manager")
        self.resize(800, 600)

        layout = QVBoxLayout()

        title = QLabel("🤖 복합 작업 시뮬레이터")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # 상단 버튼
        button_layout = QHBoxLayout()
        self.btn_add_inbound = QPushButton("➕ 입고 작업 추가")
        self.btn_add_inbound.clicked.connect(self.add_inbound)
        button_layout.addWidget(self.btn_add_inbound)

        self.btn_add_outbound = QPushButton("📤 출고 작업 추가")
        self.btn_add_outbound.clicked.connect(self.add_outbound)
        button_layout.addWidget(self.btn_add_outbound)

        self.btn_assign = QPushButton("🚚 작업 할당")
        self.btn_assign.clicked.connect(self.assign_tasks)
        button_layout.addWidget(self.btn_assign)

        layout.addLayout(button_layout)

        # 로봇 완료 버튼
        self.robot_buttons_layout = QHBoxLayout()
        for robot_id in self.manager.robots.keys():
            btn = QPushButton(f"✅ 로봇 {robot_id} 완료")
            btn.clicked.connect(lambda _, rid=robot_id: self.complete_task(rid))
            self.robot_buttons_layout.addWidget(btn)
        layout.addLayout(self.robot_buttons_layout)

        # 작업 테이블
        self.table = QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["Composite ID", "Task ID", "Type", "Robot", "Status", "Location"])
        layout.addWidget(self.table)

        self.setLayout(layout)
        self.update_table()
        self.counter = 1

    def add_inbound(self):
        task = create_inbound_task(f"입고_{self.counter}")
        self.manager.add_composite_task(task)
        print(f"➕ 입고 작업 {task.task_id} 추가됨")
        self.counter += 1
        self.update_table()

    def add_outbound(self):
        task = create_outbound_task(f"출고_{self.counter}")
        self.manager.add_composite_task(task)
        print(f"📤 출고 작업 {task.task_id} 추가됨")
        self.counter += 1
        self.update_table()

    def assign_tasks(self):
        msg = self.manager.assign_tasks()
        print("🚚 할당 결과:\n" + msg)
        self.update_table()

    def complete_task(self, robot_id):
        msg = self.manager.complete_task(robot_id)
        print(f"✅ 완료 처리 결과:\n{msg}")
        self.update_table()

    def update_table(self):
        tasks = []
        for comp in self.manager.all_composite_tasks:
            for step in comp.steps:
                tasks.append(step)

        self.table.setRowCount(len(tasks))
        self.table.setColumnCount(7)  # ✅ 열 개수 증가

        headers = ["Composite ID", "Task ID", "Type", "Robot Type", "Robot", "Status", "Location"]
        self.table.setHorizontalHeaderLabels(headers)

        for row, task in enumerate(tasks):
            composite_id = next((c.task_id for c in self.manager.all_composite_tasks if task in c.steps), "")
            self.table.setItem(row, 0, QTableWidgetItem(composite_id))
            self.table.setItem(row, 1, QTableWidgetItem(str(task.task_id)))
            self.table.setItem(row, 2, QTableWidgetItem(task.task_type.name))
            self.table.setItem(row, 3, QTableWidgetItem(task.robot_type.name))  # ✅ Robot Type 열
            self.table.setItem(row, 4, QTableWidgetItem(str(task.assigned_robot) if task.assigned_robot else "-"))
            self.table.setItem(row, 5, QTableWidgetItem(task.status.name))
            self.table.setItem(row, 6, QTableWidgetItem(str(task.location)))



if __name__ == "__main__":
    app = QApplication(sys.argv)
    manager = TaskManager()
    win = TaskManagerUI(manager)
    win.show()
    app.exec()

# if __name__ == "__main__":
#     tm = TaskManager()

#     print("\n🧪 입고 작업 테스트 시작")

#     inbound1 = create_inbound_task("inbound_001", start_pos=(2, 2), display_pos=(6, 6))
#     tm.add_composite_task(inbound1)

#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # 모바일로봇 MOVE 완료
#     print(tm.assign_tasks())
#     print(tm.complete_task(2))  # 로봇팔 PICK 완료
#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # 모바일로봇 MOVE 완료
#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # WAIT_USER 완료


#     print("\n🧪 출고 작업 테스트 시작")
#     outbound1 = create_outbound_task("outbound_001", pick_pos=(3, 3), drop_pos=(9, 9))
#     tm.add_composite_task(outbound1)

#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # mobile MOVE 완료
#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # wait_user 완료
#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # mobile MOVE to 출고장
#     print(tm.assign_tasks())
#     print(tm.complete_task(2))  # arm unload
#     print(tm.assign_tasks())
#     print(tm.complete_task(1))  # finalize → idle 전환
