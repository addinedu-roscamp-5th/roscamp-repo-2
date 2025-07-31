import heapq
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import visualization

Grid = List[List[int]]
Coord = Tuple[int, int]
TimedCoord = Tuple[int, Coord]  # (time, (x, y))
ReservationTable = Dict[Tuple[int, Coord], str]  # (t, pos): agent_id


@dataclass
class Robot:
    id: str
    start: Coord
    goal: Coord


# 상하좌우 + 대기
directions = [(-1,0), (1,0), (0,-1), (0,1), (0,0)]

def heuristic(a: Coord, b: Coord) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_valid(grid: Grid, x: int, y: int) -> bool:
    rows, cols = len(grid), len(grid[0])
    return 0 <= y < rows and 0 <= x < cols and grid[y][x] == 0


def is_reserved(res_table: ReservationTable, time: int, pos: Coord) -> bool:
    return (time, pos) in res_table

def reserve_path(res_table: ReservationTable, path: List[TimedCoord], agent_id: str):
    for t, pos in path:
        res_table[(t, pos)] = agent_id


def time_expanded_a_star(grid: Grid, start: Coord, goal: Coord, res_table: ReservationTable, agent_id: str, max_time: int = 100) -> Optional[List[TimedCoord]]:
    open_set: List[Tuple[int, int, Coord, List[TimedCoord]]] = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start, [(0, start)]))
    visited: Dict[Tuple[int, Coord], bool] = {}

    while open_set:
        f_score, t, current, path = heapq.heappop(open_set)

        if (t, current) in visited:
            continue
        visited[(t, current)] = True

        if current == goal:
            # 도착 이후에도 goal에서 대기하도록 예약
            current_t = t
            for _ in range(5):
                current_t += 1
                if current_t > max_time:
                    break
                path.append((current_t, goal))
            return path

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            next_time = t + 1

            if next_time > max_time:
                continue

            # 제자리 대기
            if dx == 0 and dy == 0:
                next_pos = current

            # 유효한 이동인 경우
            elif is_valid(grid, nx, ny):
                next_pos = (nx, ny)

            # 장애물 또는 맵 밖 → 무시
            else:
                continue

            # 예약된 좌표는 스킵
            if (next_time, next_pos) in visited or is_reserved(res_table, next_time, next_pos):
                continue

            next_path = path + [(next_time, next_pos)]
            cost = next_time + heuristic(next_pos, goal)
            heapq.heappush(open_set, (cost, next_time, next_pos, next_path))



    return None  # 실패 시

rows = 12
cols = 24
test_grid = [[0 for _ in range(cols)] for _ in range(rows)]

def apply_obstacles(grid, obstacles):
    """
    장애물 좌표 리스트를 받아 test_grid에 장애물(1)을 설정하는 함수
    각 장애물은 (x1, y1, x2, y2)의 좌상단~우하단 형태로 전달됨
    """
    for x1, y1, x2, y2 in obstacles:
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                if 0 <= y < len(grid) and 0 <= x < len(grid[0]):
                    grid[y][x] = 1

obstacles = [
    (0, 2, 1, 3),   # (x1, y1, x2, y2) 출고장
    (0, 7, 1, 8),  # 입고장
    (0, 4, 1, 6),  # 로봇팔 1
    (17, 4, 22, 6),  # 로봇팔 2
    (4, 10, 5, 11),  # 충전소 1
    (7, 10, 8, 11),  # 충전소 2
    (19, 10, 20, 11),  # 충전소 3
    (11, 3, 12, 7),  # 랙 1
    (15, 3, 16, 7),  # 랙 2
    (23, 3, 23, 7),  # 랙 3
]


apply_obstacles(test_grid, obstacles)


# y,x 좌표
robots = [
    Robot(id="A1", start=(2, 8), goal=(20, 7)),
    Robot(id="A2", start=(19, 7), goal=(2, 7)),
    Robot(id="A3", start=(2, 2), goal=(9, 10)),
    Robot(id="A4", start=(18, 10), goal=(21, 3)),
    Robot(id="A5", start=(2, 6), goal=(18, 3)),
    Robot(id="A6", start=(20, 3), goal=(3, 3)),
    Robot(id="A7", start=(5, 2), goal=(0, 7)),
]

reservation_table: ReservationTable = {}
robot_paths: Dict[str, List[TimedCoord]] = {}

for robot in robots:
    path = time_expanded_a_star(test_grid, robot.start, robot.goal, reservation_table, robot.id)
    if path:
        reserve_path(reservation_table, path, robot.id)
        robot_paths[robot.id] = path
    else:
        print(f"❌ 에이전트 {robot.id} 경로 탐색 실패")

# # 출력
# for robot_id, path in robot_paths.items():
#     print(f"\n🔹 robot {robot_id} 경로:")
#     for t, pos in path:
#         print(f"t={t}: 위치={pos}")


from collections import defaultdict

# 시간 기준으로 그룹화
grouped_by_time = defaultdict(list)
for (t, pos), robot_id in reservation_table.items():
    grouped_by_time[t].append((robot_id, pos))

# 시간순으로 정렬 및 출력
print("\n📋 시간별 Reservation Table:")
for t in sorted(grouped_by_time.keys()):
    entries = grouped_by_time[t]
    formatted = ", ".join(f"{aid}{pos}" for aid, pos in entries)
    print(f"t={t:2d} | 예약자={formatted}")

# 배경 없이 사용
# visualization.plot_multi_agent_paths(test_grid, robot_paths)

# 배경 포함 (PNG 경로 지정)
visualization.plot_multi_agent_paths(test_grid, robot_paths)

