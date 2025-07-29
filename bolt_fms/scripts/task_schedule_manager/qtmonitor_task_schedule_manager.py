from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QTableWidget, QTableWidgetItem
)
from PySide6.QtCore import Qt
from enum import Enum
from queue import PriorityQueue
import time
import sys

# ==== ENUM & TASK MAPPINGS ====

class TaskType(Enum):
    INBOUND = "go to inbound"
    OUTBOUND = "go to outbound"
    DISPLAY = "go to display"
    PICKUP = "go to pickup"
    PICK = "pick item"
    PLACE = "place item"
    # CHARGE = "go to charge"

class TaskStatus(Enum):
    PENDING = "Pending"
    ASSIGNED = "Assigned"
    COMPLETED = "Completed"

class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"
    CHARGING = "Charging"
    LOWBATTERY = "LowBattery"

TASK_TYPE_TO_ROBOT = {
    # TaskType.CHARGE: "mobile",
    TaskType.PICKUP: "mobile",
    TaskType.DISPLAY: "mobile",
    TaskType.INBOUND: "mobile",
    TaskType.OUTBOUND: "mobile",
    TaskType.PICK: "robot_arm",
    TaskType.PLACE: "robot_arm",
}

# ==== CORE CLASSES ====

class Task:
    def __init__(self, task_id, task_type, location, priority=1):
        self.task_id = task_id
        self.task_type = task_type
        self.location = location
        self.priority = priority
        self.status = TaskStatus.PENDING
        self.assigned_robot = None

    def __lt__(self, other):
        return self.priority < other.priority

class Robot:
    def __init__(self, robot_id, robot_type="mobile", position=(0, 0)):
        self.id = robot_id
        self.robot_type = robot_type
        self.status = RobotStatus.IDLE
        self.position = position
        self.current_task = None

    def is_available(self):
        return self.status == RobotStatus.IDLE

def calculate_distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

class TaskManager:
    def __init__(self):
        self.task_queue = PriorityQueue()
        self.all_tasks = []
        self.robots = {
            1: Robot(1, "mobile", (0, 0)),
            2: Robot(2, "mobile", (10, 10)),
            3: Robot(3, "mobile", (5, 15)),
            4: Robot(4, "robot_arm", (3, 4)),
            5: Robot(5, "robot_arm", (3, 8))
        }

    def add_task(self, task):
        self.task_queue.put((task.priority, time.time(), task))
        self.all_tasks.append(task)

    def select_robot(self, task):
        required_type = TASK_TYPE_TO_ROBOT.get(task.task_type)
        closest = None
        min_dist = float("inf")
        for r in self.robots.values():
            if r.is_available() and r.robot_type == required_type:
                dist = calculate_distance(r.position, task.location)
                if dist < min_dist:
                    min_dist = dist
                    closest = r
        return closest

    def assign_tasks(self):
        messages = []
        while not self.task_queue.empty():
            _, _, task = self.task_queue.get()
            robot = self.select_robot(task)
            if robot:
                task.status = TaskStatus.ASSIGNED
                task.assigned_robot = robot.id
                robot.status = RobotStatus.BUSY
                robot.current_task = task
                messages.append(f"✅ Task {task.task_id} [{task.task_type.name}] → Robot {robot.id} ({robot.robot_type})")
            else:
                self.task_queue.put((task.priority, time.time(), task))
                messages.append(f"⏸ No available {TASK_TYPE_TO_ROBOT.get(task.task_type)} robot for task {task.task_id}. Requeued.")
                break
        return "\n".join(messages)

    def complete_task(self, robot_id):
        r = self.robots.get(robot_id)
        if r and r.current_task:
            r.current_task.status = TaskStatus.COMPLETED
            r.current_task = None
            r.status = RobotStatus.IDLE
            return f"✅ Robot {robot_id} completed task."
        return f"ℹ️ Robot {robot_id} has no assigned task."

# ==== QT MAIN WINDOW ====

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("📋 Task Manager with Heterogeneous Robots")
        self.resize(640, 400)
        self.manager = TaskManager()
        self.counter = 1

        layout = QVBoxLayout()
        self.label = QLabel("작업 관리 시스템 (이기종 로봇)")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)

        btn_add = QPushButton("➕ 작업 추가")
        btn_add.clicked.connect(self.add_task)
        layout.addWidget(btn_add)

        btn_assign = QPushButton("🚚 작업 할당")
        btn_assign.clicked.connect(self.assign_tasks)
        layout.addWidget(btn_assign)

        layout.addWidget(QLabel("🤖 로봇 작업 완료"))
        for robot_id in self.manager.robots.keys():
            btn = QPushButton(f"✅ 로봇 {robot_id} 완료")
            btn.clicked.connect(lambda checked, rid=robot_id: self.complete_task(rid))
            layout.addWidget(btn)

        self.table = QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["ID", "Type", "Priority", "Robot", "Status", "Location"])
        layout.addWidget(self.table)

        self.setLayout(layout)
        self.update_table()

    def add_task(self):
        types = list(TaskType)
        task_type = types[self.counter % len(types)]
        location = (5 + self.counter * 2, 5)
        t = Task(self.counter, task_type, location, priority=self.counter)
        self.manager.add_task(t)
        print(f"➕ Task {self.counter} 추가됨: {t.task_type.name}, 위치 {t.location}, 우선순위 {t.priority}")
        self.counter += 1
        self.update_table()

    def assign_tasks(self):
        msg = self.manager.assign_tasks()
        print("🚚 작업 할당 결과:\n" + msg)
        self.update_table()

    def complete_task(self, robot_id):
        msg = self.manager.complete_task(robot_id)
        print(msg)
        self.update_table()

    def update_table(self):
        tasks = self.manager.all_tasks
        self.table.setRowCount(len(tasks))
        for row, task in enumerate(tasks):
            self.table.setItem(row, 0, QTableWidgetItem(str(task.task_id)))
            self.table.setItem(row, 1, QTableWidgetItem(task.task_type.name))
            self.table.setItem(row, 2, QTableWidgetItem(str(task.priority)))
            self.table.setItem(row, 3, QTableWidgetItem(str(task.assigned_robot) if task.assigned_robot else "-"))
            self.table.setItem(row, 4, QTableWidgetItem(task.status.value))
            self.table.setItem(row, 5, QTableWidgetItem(f"{task.location}"))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    app.exec()
