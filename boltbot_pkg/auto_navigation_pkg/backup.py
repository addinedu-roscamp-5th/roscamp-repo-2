#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
이 스크립트는 qmonitor_state_machine.py를 기반으로 실제 로봇 환경에 맞게 수정된 모니터링 GUI입니다.
- 목적: 로봇의 현재 위치(camera_pose), 목표 위치(target_pose), 그리고 상태 머신 진행 상황을 시각화합니다.
- 주요 변경 사항:
  - turtlesim.msg.Pose 대신 geometry_msgs.msg.PoseStamped 메시지를 사용합니다.
  - auto_grid_and_publisher.py의 그리드 설정을 가져와 맵 영역에 그리드를 그립니다.
  - 마우스 드래그를 통해 geometry_msgs.msg.PoseStamped 형태의 target_pose를 발행합니다.
- 의존성: rclpy, geometry_msgs, std_msgs, PyQt5, matplotlib, numpy, math, threading, sys
- 참조 위치:
  - GUI 구조 및 상태 시각화: controller_tutorials/qmonitor_state_machine.py
  - PoseStamped 메시지 처리: auto_navigation_pkg/auto_navigation_node.py
  - 그리드 설정: bolt_fms/scripts/auto_grid_and_publisher.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import threading
import math
import sys
import numpy as np

# PyQt5와 matplotlib 임포트
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import FancyBboxPatch  # 라운드 테두리 처리를 위해

# auto_grid_and_publisher.py에서 가져온 그리드 설정
# 실제 환경에 맞게 이 값들을 조정해야 합니다.
REAL_MAX_WIDTH = 1.91  # 실제 환경의 최대 너비 (미터)
REAL_MAX_HEIGHT = 0.91 # 실제 환경의 최대 높이 (미터)
ROWS = 12              # 그리드 행 수
COLS = 24              # 그리드 열 수

# PoseStamped 메시지에서 x, y, yaw를 추출하는 헬퍼 함수
def extract_pose_from_msg(msg: PoseStamped):
    x = msg.pose.position.x
    y = msg.pose.position.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    yaw = 2 * math.atan2(qz, qw)
    return {'x': x, 'y': y, 'yaw': yaw}

# ROS 노드: 로봇의 현재 pose, goal_pose, state 구독 및 goal_pose 발행
class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.robot_pose = None      # 현재 로봇의 pose (camera_pose)
        self.target_pose = None     # 현재 목표 pose (target_pose)
        self.current_state = None   # 현재 상태 (문자열)
        self.guide_line_start = None # 가이드 선의 시작점 (로봇의 pose)

        # 구독자 설정: /camera_pose와 /state 토픽 구독
        self.create_subscription(PoseStamped, 'camera_pose', self.robot_pose_callback, 10)
        self.create_subscription(PoseStamped, 'target_pose', self.target_pose_callback, 10)
        self.create_subscription(String, 'state', self.state_callback, 10)

        # 퍼블리셔: 마우스 드래그로 새 target_pose 발행
        self.target_pub = self.create_publisher(PoseStamped, 'target_pose', 10)

    def robot_pose_callback(self, msg):
        self.robot_pose = extract_pose_from_msg(msg)

    def target_pose_callback(self, msg):
        self.target_pose = extract_pose_from_msg(msg)
        if self.robot_pose is not None:
            self.guide_line_start = (self.robot_pose['x'], self.robot_pose['y'])
            self.get_logger().info(
                f"가이드 라인 설정: ({self.robot_pose['x']:.2f}, {self.robot_pose['y']:.2f}) 에서 ({self.target_pose['x']:.2f}, {self.target_pose['y']:.2f}) 까지"
            )

    def state_callback(self, msg):
        self.current_state = msg.data

# ROS 스핀 함수 (별도 스레드에서 실행)
def ros_spin(node):
    rclpy.spin(node)

# 메인 GUI 윈도우 클래스
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("로봇 모니터 (그리드 및 상태 표시)")

        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # [좌측] 로봇 맵 영역
        self.figure_map = Figure()
        self.canvas_map = FigureCanvas(self.figure_map)
        self.ax_map = self.figure_map.add_subplot(111)
        main_layout.addWidget(self.canvas_map, stretch=3)

        # [우측] 상태 표시 영역
        self.figure_state = Figure(figsize=(3, 6))
        self.canvas_state = FigureCanvas(self.figure_state)
        self.ax_state = self.figure_state.add_subplot(111)
        main_layout.addWidget(self.canvas_state, stretch=1)

        self.drag_start = None
        self.drag_current = None

        # 마우스 이벤트 연결
        self.canvas_map.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas_map.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas_map.mpl_connect('button_release_event', self.on_mouse_release)

        # GUI 업데이트 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(100) # 100ms 마다 업데이트 (10Hz)

        # 컨트롤러에서 발행하는 상태와 일치 (상태 목록)
        self.state_list = ["RotateToGoal", "MoveToGoal", "RotateToFinal", "GoalReached"]

    def on_mouse_press(self, event):
        """ 마우스 클릭 시 드래그 시작점 설정 """
        if event.button == 1 and event.inaxes == self.ax_map:
            self.drag_start = (event.xdata, event.ydata)
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_move(self, event):
        """ 마우스 드래그 중 현재 위치 업데이트 """
        if self.drag_start is not None and event.inaxes == self.ax_map:
            self.drag_current = (event.xdata, event.ydata)

    def on_mouse_release(self, event):
        """ 마우스 릴리즈 시 목표 PoseStamped 메시지 발행 """
        if event.button == 1 and self.drag_start is not None and event.inaxes == self.ax_map:
            if self.drag_current is not None:
                dx = self.drag_current[0] - self.drag_start[0]
                dy = self.drag_current[1] - self.drag_start[1]
                theta = math.atan2(dy, dx) # 드래그 방향을 yaw로 사용
            else:
                theta = 0.0

            target_msg = PoseStamped()
            target_msg.header.stamp = self.node.get_clock().now().to_msg()
            target_msg.header.frame_id = 'map' # 또는 'odom' 등 적절한 frame_id
            target_msg.pose.position.x = float(self.drag_start[0])
            target_msg.pose.position.y = float(self.drag_start[1])
            target_msg.pose.position.z = 0.0
            # yaw 각도를 쿼터니언으로 변환
            target_msg.pose.orientation.z = math.sin(theta / 2.0)
            target_msg.pose.orientation.w = math.cos(theta / 2.0)

            self.node.target_pub.publish(target_msg)
            self.node.get_logger().info(
                f"새로운 목표 발행: x={target_msg.pose.position.x:.2f}, y={target_msg.pose.position.y:.2f}, yaw={math.degrees(theta):.2f}°"
            )
            self.drag_start = None
            self.drag_current = None

    def update_all(self):
        """ 맵과 상태 표시를 모두 업데이트 """
        self.update_map()
        self.update_state_display()

    def update_map(self):
        """ 맵 영역 업데이트 (로봇 위치, 목표, 그리드) """
        self.ax_map.clear()
        
        # 그리드 영역 설정 (auto_grid_and_publisher.py의 REAL_MAX_WIDTH, REAL_MAX_HEIGHT 사용)
        self.ax_map.set_xlim(0, REAL_MAX_WIDTH)
        self.ax_map.set_ylim(0, REAL_MAX_HEIGHT)
        self.ax_map.set_aspect('equal')
        self.ax_map.set_title("로봇 맵")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")

        # 그리드 라인 그리기
        for i in range(COLS + 1):
            self.ax_map.axvline(i * (REAL_MAX_WIDTH / COLS), color='lightgray', linestyle='--', linewidth=0.5)
        for i in range(ROWS + 1):
            self.ax_map.axhline(i * (REAL_MAX_HEIGHT / ROWS), color='lightgray', linestyle='--', linewidth=0.5)

        if self.node.robot_pose is not None:
            x = self.node.robot_pose['x']
            y = self.node.robot_pose['y']
            yaw = self.node.robot_pose['yaw']
            arrow_len = 0.1 # 로봇 표시 화살표 길이
            dx = arrow_len * math.cos(yaw)
            dy = arrow_len * math.sin(yaw)
            self.ax_map.arrow(x, y, dx, dy, head_width=0.05, head_length=0.05, fc='blue', ec='blue', label='로봇')
            self.ax_map.plot(x, y, 'bo', markersize=5)

        if self.node.target_pose is not None:
            gx = self.node.target_pose['x']
            gy = self.node.target_pose['y']
            gyaw = self.node.target_pose['yaw']
            self.ax_map.plot(gx, gy, 'ro', markersize=8, label='목표')
            arrow_len = 0.1
            dx = arrow_len * math.cos(gyaw)
            dy = arrow_len * math.sin(gyaw)
            self.ax_map.arrow(gx, gy, dx, dy, head_width=0.05, head_length=0.05, fc='red', ec='red')

        if self.drag_start is not None and self.drag_current is not None:
            sx, sy = self.drag_start
            cx, cy = self.drag_current
            self.ax_map.arrow(  sx, sy, cx - sx, cy - sy, head_width=0.05, head_length=0.05,
                                fc='green', ec='green', linestyle='--', label='드래그')
            
        if self.node.guide_line_start is not None and self.node.target_pose is not None:
            start_x, start_y = self.node.guide_line_start
            target_x = self.node.target_pose['x']
            target_y = self.node.target_pose['y']
            self.ax_map.plot([start_x, target_x], [start_y, target_y], 'r--', label='경로')
            # 로봇이 목표에 충분히 가까워지면 가이드 라인 초기화
            if self.node.robot_pose is not None:
                dist = math.sqrt(   (self.node.robot_pose['x'] - target_x)**2 +
                                    (self.node.robot_pose['y'] - target_y)**2)
                if dist < 0.1: # 0.1m 이내로 가까워지면 초기화
                    self.node.guide_line_start = None
        
        self.ax_map.legend(loc='upper right')
        self.canvas_map.draw()

    def update_state_display(self):
        """ 상태 표시 영역 업데이트 """
        self.ax_state.clear()
        # 오른쪽 영역: x축 0 ~ 1.5, y축 0 ~ 1
        self.ax_state.set_xlim(0, 1.5)
        self.ax_state.set_ylim(0, 1)
        self.ax_state.axis('off') # 축 숨기기
        self.ax_state.set_title("상태 머신", fontsize=12)

        block_width = 0.8 * 1.2      # 블록 너비
        block_height = 0.06 * 1.1    # 블록 높이
        spacing = 0.12             # 블록 간 간격 (화살표 길이)
        
        # 오른쪽 영역의 폭은 1.5이므로, 가로 중앙에 배치
        start_x = (1.5 - block_width) / 2

        # 블록 전체 그룹을 수직 중앙 정렬하기 위해 총 높이를 계산
        n = len(self.state_list)
        total_height = n * block_height + (n - 1) * spacing
        group_bottom = 0.5 - total_height / 2
        group_top = group_bottom + total_height

        boundaries = []
        active_color = '#A2D2FF'    # 파스텔 블루
        inactive_color = '#E9ECEF'  # 연한 파스텔 그레이

        # 블록들을 그룹 중앙 기준으로 위에서부터 차례로 배치
        for i, state in enumerate(self.state_list):
            # i=0: 최상단 블록, 각 블록의 바닥 y 좌표
            y = group_top - (i + 1) * block_height - i * spacing
            face_color = active_color if self.node.current_state == state else inactive_color
            rect = FancyBboxPatch(( start_x, y), block_width, block_height,
                                    boxstyle="round,pad=0.02",
                                    fc=face_color, ec="black", lw=1.5)
            self.ax_state.add_patch(rect)
            self.ax_state.text(  start_x + block_width/2, y + block_height/2,
                                state, horizontalalignment='center',
                                verticalalignment='center',
                                color='black', fontsize=10)
            top_center = (start_x + block_width/2, y + block_height)
            bottom_center = (start_x + block_width/2, y)
            boundaries.append((top_center, bottom_center))
        
        # 상태 간 화살표 그리기
        for i in range(len(boundaries) - 1):
            start_point = boundaries[i][1]
            end_point = boundaries[i+1][0]
            self.ax_state.annotate( "",
                                    xy=end_point, xycoords='data',
                                    xytext=start_point, textcoords='data',
                                    arrowprops=dict(arrowstyle="->", color='black'))
        self.canvas_state.draw()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    # ROS 노드를 별도의 스레드에서 스핀하여 GUI가 블록되지 않도록 함
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    ret = app.exec_() # GUI 이벤트 루프 시작

    # GUI 종료 시 ROS 노드 정리
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
