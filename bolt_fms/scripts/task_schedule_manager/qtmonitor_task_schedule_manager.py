from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox, QTableWidget, QTableWidgetItem
)
from PySide6.QtCore import Qt
from enum import Enum
from queue import PriorityQueue
import time
import sys


# === ENUM & CLASS 정의 ===
class TaskStatus(Enum):
    PENDING = "Pending"
    ASSIGNED = "Assigned"
    COMPLETED = "Completed"

class TaskType(Enum):
    PICKUP = "Pickup"
    CHARGE = "Charge"

class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"

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
    def __init__(self, robot_id, position=(0, 0)):
        self.id = robot_id
        self.status = RobotStatus.IDLE
        self.position = position
        self.current_task = None

    def is_available(self):
        return self.status == RobotStatus.IDLE


class TaskManager:
    def __init__(self):
        self.task_queue = PriorityQueue()
        self.all_tasks = []
        self.robots = {
            1: Robot(1, (0, 0)),
            2: Robot(2, (10, 10)),
            3: Robot(3, (5, 15))
        }

    def add_task(self, task):
        self.task_queue.put((task.priority, time.time(), task))
        self.all_tasks.append(task)

    def assign_tasks(self):
        messages = []
        while not self.task_queue.empty():
            _, _, task = self.task_queue.get()
            robot = self.select_robot(task.location)
            if robot:
                robot.status = RobotStatus.BUSY
                robot.current_task = task
                task.status = TaskStatus.ASSIGNED
                task.assigned_robot = robot.id
                messages.append(f"✅ Task {task.task_id} assigned to Robot {robot.id}")
            else:
                self.task_queue.put((task.priority, time.time(), task))
                messages.append("⏸ No robot available.")
                break
        return "\n".join(messages)

    def select_robot(self, location):
        closest = None
        min_dist = float("inf")
        for r in self.robots.values():
            if r.is_available():
                dist = (r.position[0]-location[0])**2 + (r.position[1]-location[1])**2
                if dist < min_dist:
                    min_dist = dist
                    closest = r
        return closest

    def complete_task(self, robot_id):
        r = self.robots.get(robot_id)
        if r and r.current_task:
            r.current_task.status = TaskStatus.COMPLETED
            r.current_task = None
            r.status = RobotStatus.IDLE
            return f"✅ Robot {robot_id} completed its task."
        return f"ℹ️ Robot {robot_id} has no task."


# === MAIN UI ===
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("📋 Task Manager")
        self.resize(560, 400)
        self.manager = TaskManager()
        self.counter = 1

        layout = QVBoxLayout()

        self.label = QLabel("작업 관리 시스템")
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
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["ID", "Type", "Priority", "Robot", "Status"])
        layout.addWidget(self.table)

        self.setLayout(layout)
        self.update_table()

    def add_task(self):
        t = Task(self.counter, TaskType.PICKUP, (5 + self.counter * 2, 5), priority=self.counter)
        self.manager.add_task(t)
        print(f"➕ Task {self.counter} 추가됨: {t.task_type.name}, 우선순위 {t.priority}")
        self.counter += 1
        self.update_table()

    def assign_tasks(self):
        msg = self.manager.assign_tasks()
        print("🚚 작업 할당 결과:")
        print(msg)
        self.update_table()

    def complete_task(self, robot_id):
        msg = self.manager.complete_task(robot_id)
        print(f"✅ 로봇 {robot_id} 작업 완료 처리:")
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    app.exec()
