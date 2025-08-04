from queue import PriorityQueue
from enum import Enum
import time

class TaskType(Enum):
    INBOUND = "go to inbound"
    PICKUP = "go to pickup"
    DISPLAY = "go to display"
    OUTBOUND = "go to outbound"
    CHARGE = "go to charge"
    PICK = "pick item"
    PLACE = "place item"

class TaskStatus(Enum):
    PENDING = "Pending"
    ASSIGNED = "Assigned"
    IN_PROGRESS = "In Progress"
    COMPLETED = "Completed"
    FAILED = "Failed"
    CANCELLED = "Cancelled"


# 📦 TaskType ↔ RobotType 매핑
TASK_TYPE_TO_ROBOT = {
    TaskType.PICKUP: "mobile",
    TaskType.PLACE: "robot_arm",
    TaskType.CHARGE: "mobile",
    TaskType.DISPLAY: "robot_arm",
    TaskType.INBOUND: "mobile",
    TaskType.OUTBOUND: "mobile",
    TaskType.PICK: "robot_arm",
}

class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"
    CHARGING = "Charging"

class Task:
    def __init__(self, task_id, task_type: TaskType, location, priority=1, deadline=None):
        self.task_id = task_id
        self.task_type = task_type
        self.location = location
        self.priority = priority
        self.status = TaskStatus.PENDING
        self.assigned_robot = None

    def __lt__(self, other):
        return self.priority < other.priority

class Robot:
    def __init__(self, robot_id, robot_type="mobile", status=RobotStatus.IDLE, position=(0, 0)):
        self.id = robot_id
        self.robot_type = robot_type
        self.status = status
        self.position = position
        self.current_task = None

    def is_available(self):
        return (self.status == RobotStatus.IDLE and
            (self.current_task is None or self.current_task.status in [TaskStatus.COMPLETED, TaskStatus.CANCELLED]))

def calculate_distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

class TaskManager:
    def __init__(self):
        self.task_queue = PriorityQueue()
        self.robots = {
            1: Robot(robot_id=1, robot_type="mobile"),
            2: Robot(robot_id=2, robot_type="robot_arm")
        }

    def add_task(self, task: Task):
        self.task_queue.put((task.priority, time.time(), task))

    def select_robot(self, task: Task):
        required_type = TASK_TYPE_TO_ROBOT.get(task.task_type)
        closest_robot = None
        min_distance = float('inf')

        for robot in self.robots.values():
            if robot.is_available() and robot.robot_type == required_type:
                distance = calculate_distance(robot.position, task.location)
                if distance < min_distance:
                    min_distance = distance
                    closest_robot = robot

        return closest_robot

    def send_to_robot(self, robot: Robot, task: Task):
        robot.status = RobotStatus.BUSY
        print(f"✅ Task {task.task_id} of type [{task.task_type.name}] assigned to Robot {robot.id} ({robot.robot_type})")

    def assign_tasks(self):
        while not self.task_queue.empty():
            _, _, task = self.task_queue.get()
            robot = self.select_robot(task)
            if robot:
                task.assigned_robot = robot.id
                task.status = TaskStatus.ASSIGNED
                robot.status = RobotStatus.BUSY
                robot.current_task = task
                self.send_to_robot(robot, task)
            else:
                print(f"⏸ No available robot for task {task.task_id} ({task.task_type.name}). Requeuing.")
                self.task_queue.put((task.priority, time.time(), task))
                break

    def assign_task_to_robot(self, task: Task, robot_id: int):
        robot = self.robots.get(robot_id)
        if not robot:
            print(f"❌ Robot {robot_id} does not exist.")
            return

        expected_type = TASK_TYPE_TO_ROBOT.get(task.task_type)
        if robot.robot_type != expected_type:
            print(f"⚠️ Robot {robot.id} type mismatch: needs [{expected_type}], but is [{robot.robot_type}]")
            return

        if not robot.is_available():
            print(f"⏸ Robot {robot_id} is not available (status: {robot.status.name}).")
            return

        task.assigned_robot = robot.id
        task.status = TaskStatus.ASSIGNED
        robot.status = RobotStatus.BUSY
        robot.current_task = task
        self.send_to_robot(robot, task)

    def complete_task(self, robot_id):
        robot = self.robots.get(robot_id)
        if not robot:
            print(f"❌ Robot {robot_id} does not exist.")
            return

        task = robot.current_task
        if task:
            task.status = TaskStatus.COMPLETED
            print(f"✅ Task {task.task_id} completed by Robot {robot.id}.")
            robot.current_task = None
        else:
            print(f"ℹ️ Robot {robot.id} had no assigned task.")

        robot.status = RobotStatus.IDLE
        print(f"🤖 Robot {robot.id} is now IDLE and ready for new tasks.")

if __name__ == "__main__":
    print("\n🧪 [테스트 시작] TaskManager 테스트 실행 중...\n")

    task_manager = TaskManager()

    task_manager.robots[1].position = (5, 5)
    task_manager.robots[2].position = (15, 15)

    task1 = Task(1, TaskType.PICKUP, (10, 10), priority=2)
    task2 = Task(2, TaskType.DISPLAY, (6, 6), priority=1)
    task3 = Task(3, TaskType.CHARGE, (20, 20), priority=3)

    print("📦 작업 추가 중...")
    task_manager.add_task(task1)
    task_manager.add_task(task2)
    task_manager.add_task(task3)

    print("\n🤖 현재 로봇 상태:")
    for robot in task_manager.robots.values():
        print(f" - Robot {robot.id}: position={robot.position}, type={robot.robot_type}, status={robot.status.name}")

    print("\n🚚 작업 할당 시작")
    task_manager.assign_tasks()

    print("\n📋 작업 상태 확인:")
    for task in [task1, task2, task3]:
        print(f" - Task {task.task_id}: status={task.status.name}, assigned_robot={task.assigned_robot}")

    task4 = Task(4, TaskType.PLACE, (0, 0), priority=1)
    print("\n➕ 새로운 작업 추가 (모든 로봇이 BUSY 상태일 때)")
    task_manager.add_task(task4)

    print("\n🔄 작업 완료 시뮬레이션 중...")
    task_manager.complete_task(robot_id=1)
    task_manager.complete_task(robot_id=2)

    print("\n⚡ 특정 로봇에게 충전 작업 강제 할당")
    task5 = Task(5, TaskType.CHARGE, (1, 1), priority=99)
    task_manager.assign_task_to_robot(task5, robot_id=1)

    print("\n🚚 다시 할당 시도")
    task_manager.assign_tasks()

    print("\n✅ [테스트 종료]")