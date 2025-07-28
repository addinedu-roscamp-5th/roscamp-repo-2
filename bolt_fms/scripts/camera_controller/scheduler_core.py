from enum import Enum
from datetime import datetime

class TaskType(Enum):
    INBOUND = "입고"
    OUTBOUND = "출고"
    DISPLAY = "진열"
    PICK = "집품"
    CHARGE = "충전"

class Task:
    def __init__(self, task_id, task_type, location, priority=1, deadline=None):
        self.task_id = task_id
        self.task_type = task_type
        self.location = location
        self.priority = priority
        self.deadline = deadline or datetime.max
        self.assigned_robot = None
        self.status = "대기"  # 대기, 진행중, 완료

class RobotStatus(Enum):
    IDLE = "대기"
    BUSY = "작업중"
    CHARGING = "충전중"
    ERROR = "에러"

class Robot:
    def __init__(self, robot_id, current_location):
        self.robot_id = robot_id
        self.status = RobotStatus.IDLE
        self.current_location = current_location
        self.task = None
        self.total_tasks_handled = 0  # ✅ 부하 균형용 필드 추가

    def assign_task(self, task):
        self.task = task
        self.status = RobotStatus.BUSY
        self.total_tasks_handled += 1  # ✅ 작업 처리 누적
        task.assigned_robot = self.robot_id
        task.status = "진행중"

class TaskScheduler:
    def __init__(self):
        self.task_queue = []
        self.robots = []

    def add_task(self, task):
        self.task_queue.append(task)
        self.task_queue.sort(key=lambda t: (t.priority, t.deadline))

    def register_robot(self, robot):
        self.robots.append(robot)

    def schedule(self):
        if not self.robots:
            print("❌ 로봇 없음")
            return

        # 1. 작업 우선순위 + 마감시간 고려한 정렬
        self.task_queue.sort(key=lambda t: (t.priority, t.deadline))

        for task in list(self.task_queue):
            idle_robots = [r for r in self.robots if r.status == RobotStatus.IDLE]
            if not idle_robots:
                print("⚠️ 할당 실패: 모든 로봇이 바쁨")
                break

            # 2. 각 로봇에 대해 복합 스코어 계산
            def score(robot):
                dist = self.distance(robot.current_location, task.location)
                load = robot.total_tasks_handled
                return dist + load * 5  # ✅ 거리 + 부하 가중치 점수 계산

            selected_robot = min(idle_robots, key=score)
            selected_robot.assign_task(task)
            print(f"[복합할당] 작업 {task.task_id} → 로봇 {selected_robot.robot_id}")
            self.task_queue.remove(task)

    def distance(self, loc1, loc2):
        # 맨해튼 거리 계산
        return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])
