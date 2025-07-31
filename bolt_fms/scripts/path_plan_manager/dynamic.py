import heapq
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass, field
import math
import time
from threading import Lock
from collections import defaultdict
# import visualization

Grid = List[List[int]]
Coord = Tuple[int, int]
TimedCoord = Tuple[float, Coord]  # (time, (x, y))


@dataclass
class Robot:
    id: str
    start: Coord
    goal: Coord
    size: float = 1.0
    max_speed: float = 1.0
    arrival_time: float = 0.0  # 로봇이 시스템에 추가된 시간
    priority: int = 1  # 우선순위 (1=높음, 5=낮음)
    current_pos: Optional[Coord] = None
    current_time: float = 0.0
    path: List[TimedCoord] = field(default_factory=list)
    is_active: bool = True
    
    def get_occupied_cells(self, pos: Coord) -> Set[Coord]:
        """로봇이 특정 위치에 있을 때 점유하는 모든 셀을 반환"""
        x, y = pos
        occupied = set()
        size_range = int(math.ceil(self.size))
        for dx in range(-size_range, size_range + 1):
            for dy in range(-size_range, size_range + 1):
                if dx*dx + dy*dy <= self.size * self.size:
                    occupied.add((x + dx, y + dy))
        return occupied


class DynamicPathPlanner:
    def __init__(self, grid: Grid):
        self.grid = grid
        self.robots: Dict[str, Robot] = {}
        self.reservation_table: Dict[Tuple[float, Coord], str] = {}
        self.global_time = 0.0
        self.lock = Lock()  # 스레드 안전성을 위한 락
        self.planning_horizon = 50.0  # 계획 수립 시간 범위
        
    def add_robot(self, robot: Robot, current_time: float = None) -> bool:
        """실시간으로 새 로봇을 추가"""
        with self.lock:
            if current_time is None:
                current_time = self.global_time
                
            robot.arrival_time = current_time
            robot.current_time = current_time
            robot.current_pos = robot.start
            
            print(f"🤖 로봇 {robot.id} 추가 요청 (t={current_time:.1f})")
            
            # 즉시 경로 계획
            success = self._plan_robot_path(robot, current_time)
            
            if success:
                self.robots[robot.id] = robot
                print(f"  ✅ 로봇 {robot.id} 성공적으로 추가됨")
                return True
            else:
                print(f"  ❌ 로봇 {robot.id} 추가 실패 - 안전한 경로 없음")
                return False
    
    def remove_robot(self, robot_id: str):
        """로봇을 시스템에서 제거"""
        with self.lock:
            if robot_id in self.robots:
                robot = self.robots[robot_id]
                robot.is_active = False
                # 해당 로봇의 미래 예약을 모두 삭제
                self._clear_future_reservations(robot_id, self.global_time)
                del self.robots[robot_id]
                print(f"🗑️ 로봇 {robot_id} 제거됨")
    
    def update_global_time(self, new_time: float):
        """글로벌 시간 업데이트 및 과거 예약 정리"""
        with self.lock:
            self.global_time = new_time
            self._cleanup_old_reservations(new_time)
            self._update_robot_positions(new_time)
    
    def _cleanup_old_reservations(self, current_time: float):
        """과거 예약 기록 정리"""
        to_remove = []
        for (t, pos), robot_id in self.reservation_table.items():
            if t < current_time - 1.0:  # 1초 이전 기록 삭제
                to_remove.append((t, pos))
        
        for key in to_remove:
            del self.reservation_table[key]
    
    def _update_robot_positions(self, current_time: float):
        """모든 로봇의 현재 위치 업데이트"""
        for robot in self.robots.values():
            robot.current_pos = self._get_robot_position_at_time(robot, current_time)
            robot.current_time = current_time
    
    def _get_robot_position_at_time(self, robot: Robot, target_time: float) -> Coord:
        """특정 시간에서의 로봇 위치 계산"""
        if not robot.path:
            return robot.start
            
        for i, (t, pos) in enumerate(robot.path):
            if t >= target_time:
                if i == 0:
                    return pos
                # 선형 보간 (실제로는 더 정교한 방법 사용 가능)
                prev_t, prev_pos = robot.path[i-1]
                if t == prev_t:
                    return pos
                ratio = (target_time - prev_t) / (t - prev_t)
                # 단순화를 위해 가장 가까운 위치 반환
                return pos if ratio > 0.5 else prev_pos
        
        # 경로 끝에 도달한 경우
        return robot.path[-1][1] if robot.path else robot.start
    
    def _clear_future_reservations(self, robot_id: str, from_time: float):
        """특정 로봇의 미래 예약 삭제"""
        to_remove = []
        for (t, pos), rid in self.reservation_table.items():
            if rid == robot_id and t >= from_time:
                to_remove.append((t, pos))
        
        for key in to_remove:
            del self.reservation_table[key]
    
    def _plan_robot_path(self, robot: Robot, start_time: float) -> bool:
        """단일 로봇의 경로 계획"""
        # 기존 예약 정리
        if robot.id in self.robots:
            self._clear_future_reservations(robot.id, start_time)
        
        # 현재 로봇들의 위치 정보 수집
        robot_positions = {}
        for rid, r in self.robots.items():
            if rid != robot.id and r.is_active and r.path:
                robot_positions[rid] = {t: pos for t, pos in r.path if t >= start_time}
        
        # A* 경로 탐색
        path = self._time_expanded_a_star(
            robot, start_time, robot_positions, self.planning_horizon
        )
        
        if path:
            robot.path = path
            self._reserve_path(robot, path)
            return True
        
        return False
    
    def replan_robot(self, robot_id: str, new_goal: Optional[Coord] = None) -> bool:
        """로봇의 경로 재계획"""
        with self.lock:
            if robot_id not in self.robots:
                return False
                
            robot = self.robots[robot_id]
            if new_goal:
                robot.goal = new_goal
            
            current_pos = self._get_robot_position_at_time(robot, self.global_time)
            robot.start = current_pos
            
            print(f"🔄 로봇 {robot_id} 재계획 중... (현재위치: {current_pos})")
            
            return self._plan_robot_path(robot, self.global_time)
    
    def _time_expanded_a_star(self, robot: Robot, start_time: float, 
                             robot_positions: Dict[str, Dict[float, Coord]], 
                             max_time: float) -> Optional[List[TimedCoord]]:
        """시간 확장 A* 알고리즘"""
        directions = [(-1,0), (1,0), (0,-1), (0,1), (0,0)]
        
        open_set = []
        start_pos = robot.current_pos if robot.current_pos else robot.start
        heapq.heappush(open_set, (
            self._heuristic(start_pos, robot.goal) + robot.priority,
            start_time, start_pos, [(start_time, start_pos)]
        ))
        
        visited = set()
        
        while open_set:
            f_score, t, current, path = heapq.heappop(open_set)
            
            state_key = (round(t, 1), current)
            if state_key in visited:
                continue
            visited.add(state_key)
            
            if current == robot.goal:
                # 목표 도달 후 대기
                for wait in range(3):
                    path.append((t + wait + 1, robot.goal))
                return path
            
            for dx, dy in directions:
                next_pos, move_time = self._calculate_next_move(
                    robot, current, dx, dy, t
                )
                
                if next_pos is None or t + move_time > max_time:
                    continue
                
                next_time = t + move_time
                next_state = (round(next_time, 1), next_pos)
                
                if (next_state in visited or 
                    self._is_position_blocked(robot, next_time, next_pos, robot_positions)):
                    continue
                
                next_path = path + [(next_time, next_pos)]
                cost = (next_time + self._heuristic(next_pos, robot.goal) + 
                       robot.priority * 0.1)
                
                heapq.heappush(open_set, (cost, next_time, next_pos, next_path))
        
        return None
    
    def _calculate_next_move(self, robot: Robot, current: Coord, dx: int, dy: int, 
                           current_time: float) -> Tuple[Optional[Coord], float]:
        """다음 이동 위치와 소요 시간 계산"""
        if dx == 0 and dy == 0:  # 대기
            return current, 0.5 / robot.max_speed
        
        nx, ny = current[0] + dx, current[1] + dy
        next_pos = (nx, ny)
        
        # 로봇이 점유할 모든 셀 검사
        occupied_cells = robot.get_occupied_cells(next_pos)
        for cell_x, cell_y in occupied_cells:
            if not self._is_valid_cell(cell_x, cell_y):
                return None, 0
        
        move_distance = math.sqrt(dx*dx + dy*dy)
        move_time = move_distance / robot.max_speed
        
        return next_pos, move_time
    
    def _is_valid_cell(self, x: int, y: int) -> bool:
        """셀이 유효한지 확인"""
        rows, cols = len(self.grid), len(self.grid[0])
        return 0 <= y < rows and 0 <= x < cols and self.grid[y][x] == 0
    
    def _is_position_blocked(self, robot: Robot, time: float, pos: Coord,
                           robot_positions: Dict[str, Dict[float, Coord]]) -> bool:
        """위치가 차단되었는지 확인"""
        occupied_cells = robot.get_occupied_cells(pos)
        time_tolerance = 0.5 / robot.max_speed
        
        # 예약 테이블 확인
        for check_time in [time - time_tolerance, time, time + time_tolerance]:
            if check_time < 0:
                continue
            for cell in occupied_cells:
                if ((check_time, cell) in self.reservation_table and 
                    self.reservation_table[(check_time, cell)] != robot.id):
                    return True
        
        # 다른 로봇과의 충돌 확인
        for other_id, positions in robot_positions.items():
            if other_id == robot.id:
                continue
            for other_time, other_pos in positions.items():
                if abs(other_time - time) <= time_tolerance:
                    other_robot = self.robots.get(other_id)
                    if other_robot and self._check_collision(robot, pos, other_robot, other_pos):
                        return True
        
        return False
    
    def _check_collision(self, robot1: Robot, pos1: Coord, robot2: Robot, pos2: Coord) -> bool:
        """두 로봇 간 충돌 확인"""
        occupied1 = robot1.get_occupied_cells(pos1)
        occupied2 = robot2.get_occupied_cells(pos2)
        return bool(occupied1 & occupied2)
    
    def _reserve_path(self, robot: Robot, path: List[TimedCoord]):
        """경로를 예약 테이블에 등록"""
        for t, pos in path:
            occupied_cells = robot.get_occupied_cells(pos)
            time_duration = 1.0 / robot.max_speed
            
            for dt in [0, time_duration * 0.5, time_duration]:
                reserve_time = t + dt
                for cell in occupied_cells:
                    self.reservation_table[(reserve_time, cell)] = robot.id
    
    def _heuristic(self, a: Coord, b: Coord) -> float:
        """휴리스틱 함수"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_system_status(self) -> Dict:
        """시스템 현재 상태 반환"""
        with self.lock:
            active_robots = len([r for r in self.robots.values() if r.is_active])
            reservations = len(self.reservation_table)
            
            return {
                'current_time': self.global_time,
                'active_robots': active_robots,
                'total_reservations': reservations,
                'robots': {rid: {
                    'current_pos': r.current_pos,
                    'goal': r.goal,
                    'is_active': r.is_active
                } for rid, r in self.robots.items()}
            }


# 사용 예시 및 테스트
def test_dynamic_planning():
    # 그리드 설정
    rows, cols = 12, 24
    test_grid = [[0 for _ in range(cols)] for _ in range(rows)]
    
    # 장애물 설정
    obstacles = [
        (0, 2, 1, 3), (0, 7, 1, 8), (0, 4, 1, 6),
        (17, 4, 22, 6), (4, 10, 5, 11), (7, 10, 8, 11),
        (19, 10, 20, 11), (11, 3, 12, 7), (15, 3, 16, 7), (23, 3, 23, 7)
    ]
    
    for x1, y1, x2, y2 in obstacles:
        for y in range(y1, y2 + 1):
            for x in range(x1, x2 + 1):
                if 0 <= y < rows and 0 <= x < cols:
                    test_grid[y][x] = 1
    
    # 동적 경로 계획기 생성
    planner = DynamicPathPlanner(test_grid)
    
    print("🚀 동적 다중 로봇 경로 계획 시스템 시작")
    print("=" * 50)
    
    # 시나리오 1: 초기 로봇들 추가
    initial_robots = [
        Robot("A1", (2, 8), (20, 7), size=0.8, max_speed=1.2, priority=1),
        Robot("A2", (19, 7), (2, 7), size=1.2, max_speed=0.8, priority=2),
        Robot("A3", (2, 2), (9, 10), size=1.0, max_speed=1.0, priority=1),
    ]
    
    for robot in initial_robots:
        planner.add_robot(robot, 0.0)
    
    # 시간 진행 시뮬레이션
    for t in [5.0, 10.0, 15.0]:
        print(f"\n⏰ 시간 {t}초로 업데이트")
        planner.update_global_time(t)
        
        status = planner.get_system_status()
        print(f"  활성 로봇: {status['active_robots']}개")
        
        # 새로운 로봇 동적 추가
        if t == 5.0:
            new_robot = Robot("B1", (18, 10), (21, 3), size=0.6, max_speed=1.5, priority=3)
            planner.add_robot(new_robot, t)
        
        elif t == 10.0:
            emergency_robot = Robot("E1", (5, 2), (0, 7), size=1.0, max_speed=1.0, priority=1)
            planner.add_robot(emergency_robot, t)
        
        elif t == 15.0:
            # 기존 로봇의 목표 변경
            if planner.replan_robot("A1", new_goal=(3, 3)):
                print("  🎯 A1 로봇 목표 변경됨")
    
    # 최종 상태 출력
    print(f"\n📊 최종 시스템 상태:")
    final_status = planner.get_system_status()
    for robot_id, info in final_status['robots'].items():
        print(f"  {robot_id}: {info['current_pos']} → {info['goal']} "
              f"({'활성' if info['is_active'] else '비활성'})")
    
    return planner


if __name__ == "__main__":
    planner = test_dynamic_planning()
    print(f"\n🎉 동적 다중 로봇 경로 계획 시스템 테스트 완료!")