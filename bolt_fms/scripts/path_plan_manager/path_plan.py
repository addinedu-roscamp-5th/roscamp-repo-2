import heapq
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass
import math
import visualization

Grid = List[List[int]]
Coord = Tuple[int, int]
TimedCoord = Tuple[float, Coord]  # (time, (x, y)) - time을 float로 변경
ReservationTable = Dict[Tuple[float, Coord], str]  # (t, pos): agent_id


@dataclass
class Robot:
    id: str
    start: Coord
    goal: Coord
    size: float = 1.0  # 로봇 크기 (반지름)
    max_speed: float = 1.0  # 최대 속도 (칸/초)
    
    def get_occupied_cells(self, pos: Coord) -> Set[Coord]:
        """로봇이 특정 위치에 있을 때 점유하는 모든 셀을 반환"""
        x, y = pos
        occupied = set()
        
        # 로봇 크기에 따라 점유 영역 계산
        size_range = int(math.ceil(self.size))
        for dx in range(-size_range, size_range + 1):
            for dy in range(-size_range, size_range + 1):
                # 유클리드 거리로 원형 영역 계산
                if dx*dx + dy*dy <= self.size * self.size:
                    occupied.add((x + dx, y + dy))
        
        return occupied


# 상하좌우 + 대기
directions = [(-1,0), (1,0), (0,-1), (0,1), (0,0)]

def heuristic(a: Coord, b: Coord) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_valid(grid: Grid, x: int, y: int) -> bool:
    rows, cols = len(grid), len(grid[0])
    return 0 <= y < rows and 0 <= x < cols and grid[y][x] == 0

def check_robot_collision(robot1: Robot, pos1: Coord, robot2: Robot, pos2: Coord) -> bool:
    """두 로봇이 충돌하는지 확인"""
    occupied1 = robot1.get_occupied_cells(pos1)
    occupied2 = robot2.get_occupied_cells(pos2)
    return bool(occupied1 & occupied2)  # 교집합이 있으면 충돌

def is_reserved_with_size(res_table: ReservationTable, robot: Robot, time: float, pos: Coord, 
                         robot_positions: Dict[str, Dict[float, Coord]]) -> bool:
    """로봇 크기를 고려한 예약 체크"""
    occupied_cells = robot.get_occupied_cells(pos)
    
    # 시간 범위 확인 (로봇 속도에 따른 점유 시간)
    time_tolerance = 0.5 / robot.max_speed  # 속도가 느릴수록 더 오래 점유
    
    for check_time in [time - time_tolerance, time, time + time_tolerance]:
        if check_time < 0:
            continue
            
        for cell in occupied_cells:
            # 기본 예약 테이블 확인
            if (check_time, cell) in res_table and res_table[(check_time, cell)] != robot.id:
                return True
    
    # 다른 로봇들과의 충돌 확인
    for other_robot_id, positions in robot_positions.items():
        if other_robot_id == robot.id:
            continue
            
        # 해당 시간대의 다른 로봇 위치 찾기
        for other_time, other_pos in positions.items():
            if abs(other_time - time) <= time_tolerance:
                # 다른 로봇 정보 가져오기 (실제로는 robots dict에서)
                other_robot = Robot(id=other_robot_id, start=(0,0), goal=(0,0), 
                                  size=1.0, max_speed=1.0)  # 임시값
                if check_robot_collision(robot, pos, other_robot, other_pos):
                    return True
    
    return False

def reserve_path_with_size(res_table: ReservationTable, path: List[TimedCoord], robot: Robot):
    """로봇 크기를 고려한 경로 예약"""
    for t, pos in path:
        occupied_cells = robot.get_occupied_cells(pos)
        time_duration = 1.0 / robot.max_speed  # 한 칸 이동하는데 걸리는 시간
        
        # 점유 시간 동안 모든 셀 예약
        for dt in [0, time_duration * 0.5, time_duration]:
            reserve_time = t + dt
            for cell in occupied_cells:
                res_table[(reserve_time, cell)] = robot.id

def calculate_movement_time(robot: Robot, distance: float = 1.0) -> float:
    """로봇의 속도에 따른 이동 시간 계산"""
    return distance / robot.max_speed

def time_expanded_a_star_with_size(grid: Grid, robot: Robot, res_table: ReservationTable, 
                                  robot_positions: Dict[str, Dict[float, Coord]], 
                                  max_time: float = 100.0) -> Optional[List[TimedCoord]]:
    
    open_set: List[Tuple[float, float, Coord, List[TimedCoord]]] = []
    start_time = 0.0
    heapq.heappush(open_set, (heuristic(robot.start, robot.goal), start_time, robot.start, [(start_time, robot.start)]))
    visited: Dict[Tuple[float, Coord], bool] = {}

    while open_set:
        f_score, t, current, path = heapq.heappop(open_set)

        state_key = (round(t, 2), current)  # 시간 반올림으로 상태 키 생성
        if state_key in visited:
            continue
        visited[state_key] = True

        if current == robot.goal:
            # 도착 이후에도 goal에서 대기하도록 예약
            wait_time = calculate_movement_time(robot)
            for wait in range(5):
                path.append((t + (wait + 1) * wait_time, robot.goal))
            return path

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            
            # 이동 거리 계산
            if dx == 0 and dy == 0:  # 대기
                move_distance = 0
                next_pos = current
            else:
                move_distance = math.sqrt(dx*dx + dy*dy)
                next_pos = (nx, ny)
            
            # 이동 시간 계산
            if move_distance == 0:
                next_time = t + 0.1  # 대기 시간
            else:
                next_time = t + calculate_movement_time(robot, move_distance)
            
            if next_time > max_time:
                continue

            # 제자리 대기가 아닌 경우 유효성 검사
            if move_distance > 0:
                # 로봇이 점유할 모든 셀이 유효한지 확인
                occupied_cells = robot.get_occupied_cells(next_pos)
                valid_move = True
                for cell_x, cell_y in occupied_cells:
                    if not is_valid(grid, cell_x, cell_y):
                        valid_move = False
                        break
                
                if not valid_move:
                    continue

            # 예약된 좌표인지 확인 (크기 고려)
            next_state_key = (round(next_time, 2), next_pos)
            if (next_state_key in visited or 
                is_reserved_with_size(res_table, robot, next_time, next_pos, robot_positions)):
                continue

            next_path = path + [(next_time, next_pos)]
            cost = next_time + heuristic(next_pos, robot.goal)
            heapq.heappush(open_set, (cost, next_time, next_pos, next_path))

    return None  # 실패 시


# 테스트 설정
rows = 12
cols = 24
test_grid = [[0 for _ in range(cols)] for _ in range(rows)]

def apply_obstacles(grid, obstacles):
    """장애물 좌표 리스트를 받아 test_grid에 장애물(1)을 설정하는 함수"""
    for x1, y1, x2, y2 in obstacles:
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                if 0 <= y < len(grid) and 0 <= x < len(grid[0]):
                    grid[y][x] = 1

obstacles = [
    (0, 2, 1, 3),   # 출고장
    (0, 7, 1, 8),   # 입고장
    (0, 4, 1, 6),   # 로봇팔 1
    (17, 4, 22, 6), # 로봇팔 2
    (4, 10, 5, 11), # 충전소 1
    (7, 10, 8, 11), # 충전소 2
    (19, 10, 20, 11), # 충전소 3
    (11, 3, 12, 7), # 랙 1
    (15, 3, 16, 7), # 랙 2
    (23, 3, 23, 7), # 랙 3
]

apply_obstacles(test_grid, obstacles)

# 다양한 크기와 속도를 가진 로봇들
robots = [
    Robot(id="A1", start=(2, 8), goal=(20, 7), size=0.8, max_speed=1.2),  # 작고 빠른 로봇
    Robot(id="A2", start=(19, 7), goal=(2, 7), size=1.2, max_speed=0.8),  # 크고 느린 로봇
    Robot(id="A3", start=(2, 2), goal=(9, 10), size=1.0, max_speed=1.0),  # 표준 로봇
    Robot(id="A4", start=(18, 10), goal=(21, 3), size=0.6, max_speed=1.5), # 매우 작고 빠른 로봇
    Robot(id="A5", start=(2, 6), goal=(18, 3), size=1.1, max_speed=0.9),  # 약간 크고 약간 느린 로봇
    Robot(id="A6", start=(20, 3), goal=(3, 3), size=0.9, max_speed=1.1),  # 약간 작고 약간 빠른 로봇
    Robot(id="A7", start=(5, 2), goal=(0, 7), size=1.0, max_speed=1.0),   # 표준 로봇
]

# 경로 계획 실행
reservation_table: ReservationTable = {}
robot_paths: Dict[str, List[TimedCoord]] = {}
robot_positions: Dict[str, Dict[float, Coord]] = {}

print("🤖 로봇 정보:")
for robot in robots:
    print(f"  {robot.id}: 크기={robot.size:.1f}, 속도={robot.max_speed:.1f}, {robot.start}→{robot.goal}")

print("\n🔍 경로 탐색 중...")

for robot in robots:
    print(f"  로봇 {robot.id} 처리 중...")
    path = time_expanded_a_star_with_size(test_grid, robot, reservation_table, robot_positions)
    
    if path:
        reserve_path_with_size(reservation_table, path, robot)
        robot_paths[robot.id] = path
        
        # 로봇 위치 정보 저장
        robot_positions[robot.id] = {t: pos for t, pos in path}
        
        print(f"    ✅ 성공! 총 {len(path)}단계, 완료시간: {path[-1][0]:.2f}초")
    else:
        print(f"    ❌ 실패!")

# 결과 출력
print(f"\n📊 결과 요약:")
print(f"  성공한 로봇: {len(robot_paths)}/{len(robots)}")

for robot_id, path in robot_paths.items():
    robot = next(r for r in robots if r.id == robot_id)
    total_time = path[-1][0] if path else 0
    distance = len([p for p in path if p != path[0]]) if len(path) > 1 else 0
    
    print(f"  {robot_id}: 거리={distance}, 시간={total_time:.2f}초, "
          f"평균속도={distance/total_time if total_time > 0 else 0:.2f}")

# 시간별 점유 현황 (처음 20초만)
from collections import defaultdict

grouped_by_time = defaultdict(list)
max_display_time = 50

for (t, pos), robot_id in reservation_table.items():
    if t <= max_display_time:
        grouped_by_time[round(t, 1)].append((robot_id, pos))

print(f"\n📋 시간별 점유 현황 (처음 {max_display_time}초):")
for t in sorted(grouped_by_time.keys())[:50]:  # 처음 50개 타임스텝만
    entries = grouped_by_time[t]
    if entries:  # 비어있지 않은 경우만
        formatted = ", ".join(f"{aid}{pos}" for aid, pos in entries[:5])  # 처음 5개만
        if len(entries) > 5:
            formatted += f" ...+{len(entries)-5}개"
        print(f"t={t:4.1f} | {formatted}")

print(f"\n🎯 시각화를 위해서는 visualization 모듈의 함수를 robot_paths와 함께 사용하세요!")

# 배경 포함 (PNG 경로 지정)
visualization.plot_multi_agent_paths(test_grid, robot_paths, background_path="/home/addinedu/colcon_ws/src/roscamp-repo-2/bolt_fms/scripts/path_plan_manager/background.png")