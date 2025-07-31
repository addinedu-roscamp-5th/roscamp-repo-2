# -*- coding: utf-8 -*-
"""
멀티 로봇 동적 경로 계획 시뮬레이터 (Qt + PySide6)
===================================================

* **설치** : `pip install PySide6`
* **실행** : `python multi_robot_qt_sim.py`

본 스크립트는 시간 확장 A* 기반 `DynamicPathPlanner` 에 Qt GUI를 씌워 실시간 다중
로봇 경로 계획 과정을 시각화합니다. 0.1 초 간격으로 글로벌 시간이 흐르며, 우측
패널에서 **로봇 직접 추가**·로그 확인이 가능합니다.
"""

###############################################################################
# 표준 라이브러리 & PySide6
###############################################################################

import sys
import math
import random
from typing import Dict

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QPen, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGroupBox,
    QGraphicsEllipseItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsTextItem,
    QGraphicsView,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

###############################################################################
# ░▒▓ 경로 계획 로직 ▓▒░  (print → self._log 로 변경)
###############################################################################

from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field
import heapq
from threading import Lock

Grid = List[List[int]]
Coord = Tuple[int, int]
TimedCoord = Tuple[float, Coord]


@dataclass
class Robot:
    id: str
    start: Coord
    goal: Coord
    size: float = 1.0
    max_speed: float = 1.0
    arrival_time: float = 0.0
    priority: int = 1
    current_pos: Optional[Coord] = None
    current_time: float = 0.0
    path: List[TimedCoord] = field(default_factory=list)
    is_active: bool = True

    def get_occupied_cells(self, pos: Coord) -> Set[Coord]:
        x, y = pos
        occupied = set()
        r = int(math.ceil(self.size))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= self.size * self.size:
                    occupied.add((x + dx, y + dy))
        return occupied


class DynamicPathPlanner:
    """시간 확장 A* 기반 다중 로봇 경로 계획기"""

    def __init__(self, grid: Grid, log_fn=print):
        self.grid = grid
        self.robots: Dict[str, Robot] = {}
        self.reservation_table: Dict[Tuple[float, Coord], str] = {}
        self.global_time = 0.0
        self.lock = Lock()
        self.planning_horizon = 50.0
        self._log = log_fn

    # ───────── 로봇 관리 ────────────────────────────────────────────────────
    def add_robot(self, robot: Robot, current_time: float | None = None) -> bool:
        with self.lock:
            if current_time is None:
                current_time = self.global_time
            robot.arrival_time = current_time
            robot.current_time = current_time
            robot.current_pos = robot.start
            self._log(f"🤖 로봇 {robot.id} 추가 요청 (t={current_time:.1f})")
            ok = self._plan_robot_path(robot, current_time)
            if ok:
                self.robots[robot.id] = robot
                self._log("  ✅ 추가 완료")
            else:
                self._log("  ❌ 추가 실패 – 안전 경로 없음")
            return ok

    def remove_robot(self, robot_id: str):
        with self.lock:
            if robot_id in self.robots:
                self._clear_future_reservations(robot_id, self.global_time)
                del self.robots[robot_id]
                self._log(f"🗑️ 로봇 {robot_id} 제거")

    # ───────── 시간 / 상태 업데이트 ─────────────────────────────────────────
    def update_global_time(self, new_time: float):
        with self.lock:
            self.global_time = new_time
            self._cleanup_old_reservations(new_time)
            self._update_robot_positions(new_time)

    def _cleanup_old_reservations(self, now: float):
        for key in [k for k in self.reservation_table if k[0] < now - 1.0]:
            del self.reservation_table[key]

    def _update_robot_positions(self, now: float):
        for r in self.robots.values():
            r.current_pos = self._pos_at(r, now)
            r.current_time = now

    # ───────── A* & 충돌 체크 ──────────────────────────────────────────────
    def _plan_robot_path(self, robot: Robot, start_time: float) -> bool:
        if robot.id in self.robots:
            self._clear_future_reservations(robot.id, start_time)
        others = {
            rid: {t: p for t, p in r.path if t >= start_time}
            for rid, r in self.robots.items() if rid != robot.id and r.is_active
        }
        path = self._time_expanded_a_star(robot, start_time, others, self.planning_horizon)
        if path:
            robot.path = path
            self._reserve(path, robot)
        return bool(path)

    def _time_expanded_a_star(self, robot: Robot, t0: float, others, tmax: float):
        DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]
        open_set = []
        start = robot.current_pos or robot.start
        heapq.heappush(open_set, (self._h(start, robot.goal), t0, start, [(t0, start)]))
        visited = set()
        while open_set:
            f, t, pos, path = heapq.heappop(open_set)
            key = (round(t, 1), pos)
            if key in visited:
                continue
            visited.add(key)
            if pos == robot.goal:
                for w in range(3):
                    path.append((t + w + 1, robot.goal))
                return path
            for dx, dy in DIRS:
                nxt, dt = self._step(robot, pos, dx, dy)
                if nxt is None or t + dt > tmax:
                    continue
                nt = t + dt
                nkey = (round(nt, 1), nxt)
                if nkey in visited or self._blocked(robot, nt, nxt, others):
                    continue
                heapq.heappush(open_set, (nt + self._h(nxt, robot.goal), nt, nxt, path + [(nt, nxt)]))
        return None
    
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

    # ────── 유틸 ───────────────────────────────────────────────────────────
    def _step(self, robot, pos, dx, dy):
        if dx == dy == 0:
            return pos, 0.5 / robot.max_speed
        nx, ny = pos[0] + dx, pos[1] + dy
        for cx, cy in robot.get_occupied_cells((nx, ny)):
            if not self._valid(cx, cy):
                return None, 0
        dist = math.hypot(dx, dy)
        return (nx, ny), dist / robot.max_speed

    def _pos_at(self, robot: Robot, t: float):
        if not robot.path:
            return robot.start
        for i, (pt, ppos) in enumerate(robot.path):
            if pt >= t:
                if i == 0:
                    return ppos
                p0t, p0 = robot.path[i - 1]
                return ppos if (t - p0t) / (pt - p0t) > 0.5 else p0
        return robot.path[-1][1]

    def _blocked(self, robot, t, pos, others):
        tol = 0.5 / robot.max_speed
        occ = robot.get_occupied_cells(pos)
        for dt in (-tol, 0, tol):
            if t + dt < 0:
                continue
            for cell in occ:
                if (t + dt, cell) in self.reservation_table and self.reservation_table[(t + dt, cell)] != robot.id:
                    return True
        for oid, poses in others.items():
            for ot, opos in poses.items():
                if abs(ot - t) <= tol:
                    orobot = self.robots.get(oid)
                    if orobot and (orobot.get_occupied_cells(opos) & occ):
                        return True
        return False

    def _reserve(self, path: List[TimedCoord], robot: Robot):
        for t, pos in path:
            for cell in robot.get_occupied_cells(pos):
                self.reservation_table[(t, cell)] = robot.id

    def _clear_future_reservations(self, rid: str, from_t: float):
        for k in [k for k, v in self.reservation_table.items() if v == rid and k[0] >= from_t]:
            del self.reservation_table[k]

    def _valid(self, x, y):
        return 0 <= y < len(self.grid) and 0 <= x < len(self.grid[0]) and self.grid[y][x] == 0

    @staticmethod
    def _h(a: Coord, b: Coord):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # GUI 조회용
    def status(self):
        return {rid: (r.current_pos, r.goal) for rid, r in self.robots.items() if r.is_active}

###############################################################################
# ░▒▓ Qt 시뮬레이터 UI ▓▒░
###############################################################################

CELL = 32
COLORS = ["#0070F3", "#FF006A", "#F5A623", "#7ED321", "#9013FE", "#50E3C2"]


class Tile(QGraphicsRectItem):
    def __init__(self, x, y, wall):
        super().__init__(0, 0, CELL, CELL)
        self.setPos(x * CELL, y * CELL)
        self.setBrush(QBrush(QColor("#333") if wall else QColor("#eee")))
        self.setPen(QPen(Qt.NoPen))


class BotItem(QGraphicsEllipseItem):
    def __init__(self, robot: Robot, color: QColor):
        r = CELL * 0.4
        super().__init__(-r, -r, 2 * r, 2 * r)
        self.setBrush(QBrush(color))
        self.setPen(QPen(Qt.black))
        self.setZValue(10)
        label = QGraphicsTextItem(robot.id)
        label.setDefaultTextColor(Qt.white)
        label.setPos(-8, -8)
        label.setParentItem(self)
        self.update_pos(robot.current_pos)

    def update_pos(self, pos):
        if pos:
            self.setPos((pos[0] + 0.5) * CELL, (pos[1] + 0.5) * CELL)


class Simulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi‑Robot Planner Sim")
        self.resize(1100, 650)

        # │ 왼쪽 : 캔버스 + 플레이 버튼
        # │ 오른쪽 : 로그 + 로봇 추가 폼
        h = QHBoxLayout(self)
        left = QVBoxLayout()
        right = QVBoxLayout()
        h.addLayout(left, 3)
        h.addLayout(right, 1)

        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing, True)
        left.addWidget(self.view)

        self.btn_play = QPushButton("▶︎ Start")
        self.btn_play.clicked.connect(self.toggle)
        left.addWidget(self.btn_play)

        # 로그 패널
        self.log = QListWidget()
        right.addWidget(self.log, 3)

        # 로봇 추가 위젯 ------------------------------------------------------
        form = QGroupBox("Add Robot")
        grid = QGridLayout(form)
        lbl = lambda t: QLabel(t)

        self.in_id = QLineEdit()
        self.in_sx = QSpinBox(); self.in_sy = QSpinBox()
        self.in_gx = QSpinBox(); self.in_gy = QSpinBox()
        for sb in (self.in_sx, self.in_sy, self.in_gx, self.in_gy):
            sb.setRange(0, 50)
        self.in_size = QDoubleSpinBox(); self.in_size.setRange(0.2, 5.0); self.in_size.setSingleStep(0.1); self.in_size.setValue(1.0)
        self.in_speed = QDoubleSpinBox(); self.in_speed.setRange(0.1, 5.0); self.in_speed.setValue(1.0)
        self.in_pri = QSpinBox(); self.in_pri.setRange(1, 5); self.in_pri.setValue(1)

        widgets = [(lbl("ID"), self.in_id),
                   (lbl("Start X"), self.in_sx), (lbl("Start Y"), self.in_sy),
                   (lbl("Goal X"), self.in_gx), (lbl("Goal Y"), self.in_gy),
                   (lbl("Size"), self.in_size), (lbl("Speed"), self.in_speed), (lbl("Priority"), self.in_pri)]
        for i, (l, w) in enumerate(widgets):
            grid.addWidget(l, i, 0); grid.addWidget(w, i, 1)

        btn_add = QPushButton("Add")
        btn_add.clicked.connect(self.on_add_robot)
        grid.addWidget(btn_add, len(widgets), 0, 1, 2)
        right.addWidget(form, 2)

        # ───── 시뮬레이터 로직 ──────────────────────────────────────────────
        self.rows, self.cols = 12, 24
        self.grid = [[0] * self.cols for _ in range(self.rows)]
        self._build_obstacles()
        self._draw_tiles()
        self.planner = DynamicPathPlanner(self.grid, log_fn=self._log)
        self.bot_items: Dict[str, BotItem] = {}
        self.time = 0.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.step)
        self._preset_events()

    # ───── UI 헬퍼 ──────────────────────────────────────────────────────────
    def _log(self, msg: str):
        self.log.addItem(QListWidgetItem(msg))
        self.log.scrollToBottom()

    def toggle(self):
        if self.timer.isActive():
            self.timer.stop(); self.btn_play.setText("▶︎ Start")
        else:
            self.timer.start(100); self.btn_play.setText("⏸ Pause")

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

    # ───── 시뮬레이션 단계 ---------------------------------------------------
    def step(self):
        self.time += 0.1
        self.planner.update_global_time(self.time)
        if self.time in self.events:
            for r in self.events[self.time]:
                self._try_add(r)
        if math.isclose(self.time, 15.0, abs_tol=1e-3):
            if self.planner.replan_robot("A1", (3, 3)):
                self._log("🎯 A1 목표 → (3,3)")
        self._sync_bots()

    # ───── 로봇 추가 ---------------------------------------------------------
    def _try_add(self, robot: Robot):
        if self.planner.add_robot(robot, self.time):
            color = QColor(random.choice(COLORS))
            item = BotItem(robot, color)
            self.scene.addItem(item)
            self.bot_items[robot.id] = item

    def on_add_robot(self):
        rid = self.in_id.text().strip()
        if not rid:
            self._log("❗ ID 입력 필요")
            return
        if rid in self.bot_items or rid in self.planner.robots:
            self._log("❗ 중복 ID")
            return
        r = Robot(
            rid,
            (self.in_sx.value(), self.in_sy.value()),
            (self.in_gx.value(), self.in_gy.value()),
            size=self.in_size.value(),
            max_speed=self.in_speed.value(),
            priority=self.in_pri.value(),
        )
        self._try_add(r)
        self.in_id.clear()

    # ───── 그래픽 동기화 -----------------------------------------------------
    def _sync_bots(self):
        for rid, (pos, goal) in self.planner.status().items():
            if rid in self.bot_items:
                self.bot_items[rid].update_pos(pos)

    # ───── 초기 맵 및 이벤트 --------------------------------------------------
    def _build_obstacles(self):
        obs = [
            (0, 2, 1, 3), (0, 7, 1, 8), (0, 4, 1, 6),
            (17, 4, 22, 6), (4, 10, 5, 11), (7, 10, 8, 11),
            (19, 10, 20, 11), (11, 3, 12, 7), (15, 3, 16, 7),
            (23, 3, 23, 7),
        ]
        for x1, y1, x2, y2 in obs:
            for y in range(y1, y2 + 1):
                for x in range(x1, x2 + 1):
                    if 0 <= y < self.rows and 0 <= x < self.cols:
                        self.grid[y][x] = 1

    def _draw_tiles(self):
        for y in range(self.rows):
            for x in range(self.cols):
                self.scene.addItem(Tile(x, y, self.grid[y][x] == 1))
        self.scene.setSceneRect(0, 0, self.cols * CELL, self.rows * CELL)

    def _preset_events(self):
        """시뮬레이션 시작 시 자동으로 발생할 로봇 추가 이벤트"""
        self.events = {
            0.0: [
                Robot("A1", (2, 8),  (20, 7), size=0.8, max_speed=1.2, priority=1),
                Robot("A2", (19, 7), (2, 7),  size=1.2, max_speed=0.8, priority=2),
                Robot("A3", (2, 2),  (9, 10), size=1.0, max_speed=1.0, priority=1),
            ],
            5.0: [
                Robot("B1", (18, 10), (21, 3), size=0.6, max_speed=1.5, priority=3),
            ],
            10.0: [
                Robot("E1", (5, 2),  (0, 7), size=1.0, max_speed=1.0, priority=1),
            ],
        }

###############################################################################
# ───── main entry ------------------------------------------------------------
###############################################################################

def main():
    app = QApplication(sys.argv)
    sim = Simulator()
    sim.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
