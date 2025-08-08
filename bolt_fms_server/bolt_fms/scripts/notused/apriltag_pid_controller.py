#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2, sin, cos, sqrt, pi
import yaml
import os

import cv2
import numpy as np
from pupil_apriltags import Detector

import sys


# === 설정 ===
TAG_ID = 2
CAMERA_INDEX = 2
VISUALIZE = True

# === 그리드 설정 ===
CAP_WIDTH = 1920
CAP_HEIGHT = 1080

TOP_LEFT = (0, 0)
TOP_RIGHT = (CAP_WIDTH - 1, 0)
BOT_LEFT = (0, CAP_HEIGHT - 1)
BOT_RIGHT = (CAP_WIDTH - 1, CAP_HEIGHT - 1)

ROWS, COLS = 12, 24
REAL_MAX_WIDTH = 1.91
REAL_MAX_HEIGHT = 0.91
REAL_ROWS = ROWS + 1
REAL_COLS = COLS + 1
REAL_WIDTH = REAL_MAX_WIDTH / REAL_COLS
REAL_HEIGHT = REAL_MAX_HEIGHT / REAL_ROWS

horizontal_divisions = ROWS - 1
vertical_divisions = COLS - 1

# === 로봇 제어 설정 ===
MAX_LINEAR_SPEED = 0.3  # m/s
MAX_ANGULAR_SPEED = 1.0  # rad/s
POSITION_TOLERANCE = 0.05  # m
ANGLE_TOLERANCE = 0.1  # rad

# === PID 제어 클래스 ===
class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, max_output=None, min_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    def reset(self):
        """PID 상태 리셋"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    def update(self, error, dt=None):
        """PID 제어 업데이트"""
        current_time = time.time()
        
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0
            
        if dt is None:
            dt = current_time - self.prev_time
            
        if dt <= 0.0:
            return 0.0
            
        # P term
        p_term = self.kp * error
        
        # I term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # D term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # 전체 출력
        output = p_term + i_term + d_term
        
        # 출력 제한
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)
            
        # 상태 업데이트
        self.prev_error = error
        self.prev_time = current_time
        
        return output

# === 로봇 상태 머신 ===
class RobotStateMachine:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state = 'IDLE'  # IDLE, MOVING, ROTATING, ARRIVED
        self.goal = None
        self.path = []
        self.current_path_index = 0
        self.last_position = None
        self.stuck_timer = 0.0
        self.stuck_threshold = 3.0  # 3초 동안 움직이지 않으면 stuck
        self.position_history = []
        
    def set_goal(self, x, y, yaw=None):
        """단일 목표 설정"""
        self.goal = (x, y, yaw if yaw is not None else 0.0)
        self.path = []
        self.state = 'MOVING'
        self.reset_stuck_detection()
        
    def set_path(self, waypoints):
        """경로 설정"""
        self.path = waypoints
        self.current_path_index = 0
        if waypoints:
            first_point = waypoints[0]
            self.goal = (first_point[0], first_point[1], 0.0)
            self.state = 'MOVING'
        self.reset_stuck_detection()
        
    def update_position(self, current_pos):
        """위치 업데이트 및 stuck 감지"""
        if current_pos is None:
            return
            
        # 위치 히스토리 업데이트
        self.position_history.append(current_pos)
        if len(self.position_history) > 10:  # 최근 10개만 유지
            self.position_history.pop(0)
            
        # Stuck 감지
        if len(self.position_history) >= 5:
            recent_positions = self.position_history[-5:]
            max_distance = 0.0
            for i in range(len(recent_positions)):
                for j in range(i+1, len(recent_positions)):
                    dist = sqrt((recent_positions[i][0] - recent_positions[j][0])**2 + 
                              (recent_positions[i][1] - recent_positions[j][1])**2)
                    max_distance = max(max_distance, dist)
            
            if max_distance < 0.02:  # 2cm 이하로만 움직임
                self.stuck_timer += 0.1
            else:
                self.stuck_timer = 0.0
                
    def is_stuck(self):
        """Stuck 상태 확인"""
        return self.stuck_timer > self.stuck_threshold
        
    def reset_stuck_detection(self):
        """Stuck 감지 리셋"""
        self.stuck_timer = 0.0
        self.position_history = []
        
    def next_waypoint(self):
        """다음 웨이포인트로 이동"""
        if self.path and self.current_path_index < len(self.path) - 1:
            self.current_path_index += 1
            next_point = self.path[self.current_path_index]
            self.goal = (next_point[0], next_point[1], 0.0)
            self.reset_stuck_detection()
            return True
        return False
        
    def stop(self):
        """정지"""
        self.state = 'IDLE'
        self.goal = None
        self.path = []
        self.reset_stuck_detection()

# === PID 설정 로더 ===
def load_pid_config(config_path="pid_config.yaml"):
    """PID 설정 파일 로드"""
    default_config = {
        'linear_pid': {'kp': 1.0, 'ki': 0.0, 'kd': 0.1},
        'angular_pid': {'kp': 2.0, 'ki': 0.0, 'kd': 0.2},
        'position_tolerance': 0.05,
        'angle_tolerance': 0.1,
        'max_linear_speed': 0.3,
        'max_angular_speed': 1.0,
        'stuck_threshold': 3.0,
        'approach_distance': 0.2  # 목표 근처에서 속도 감소 시작 거리
    }
    
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                # 기본값과 병합
                for key, value in default_config.items():
                    if key not in config:
                        config[key] = value
                return config
        else:
            # 기본 설정 파일 생성
            with open(config_path, 'w') as f:
                yaml.dump(default_config, f, default_flow_style=False)
            print(f"⚠️ 기본 PID 설정 파일 생성: {config_path}")
            return default_config
    except Exception as e:
        print(f"⚠️ PID 설정 파일 로드 실패: {e}, 기본값 사용")
        return default_config

# === ROS 노드 클래스 (PID 제어 적용) ===
class PIDPoseMultiPublisher(Node):
    def __init__(self, robot_ids, get_pose_func_map, publish_period=0.03, config_path="pid_config.yaml"):
        super().__init__("pid_multi_pose_publisher")
        
        # PID 설정 로드
        self.config = load_pid_config(config_path)
        
        # 퍼블리셔 설정
        self.pose_publishers = {}
        self.cmd_publishers = {}
        self.get_pose_func_map = get_pose_func_map
        
        # 로봇별 상태 머신과 PID 컨트롤러
        self.robot_state_machines = {}
        self.linear_pid_controllers = {}
        self.angular_pid_controllers = {}
        
        for robot_id in robot_ids:
            # Publishers
            pose_topic = f"/robot{robot_id}/camera_pose"
            self.pose_publishers[robot_id] = self.create_publisher(PoseStamped, pose_topic, 10)
            
            cmd_topic = f"/robot{robot_id}/cmd_vel"
            self.cmd_publishers[robot_id] = self.create_publisher(Twist, cmd_topic, 10)
            print("------------------------------------",cmd_topic)
            # 상태 머신
            self.robot_state_machines[robot_id] = RobotStateMachine(robot_id)
            
            # PID 컨트롤러 
            linear_config = self.config['linear_pid']
            angular_config = self.config['angular_pid']
            
            self.linear_pid_controllers[robot_id] = PIDController(
                kp=linear_config['kp'],
                ki=linear_config['ki'], 
                kd=linear_config['kd'],
                max_output=self.config['max_linear_speed'],
                min_output=-self.config['max_linear_speed']
            )
            
            self.angular_pid_controllers[robot_id] = PIDController(
                kp=angular_config['kp'],
                ki=angular_config['ki'],
                kd=angular_config['kd'], 
                max_output=self.config['max_angular_speed'],
                min_output=-self.config['max_angular_speed']
            )
            
            # 타이머
            self.create_timer(publish_period, lambda rid=robot_id: self.publish_pose(rid))
            self.create_timer(0.1, lambda rid=robot_id: self.control_robot_pid(rid))

    def set_robot_goal(self, robot_id, target_x, target_y, target_yaw=None):
        """로봇 목표 설정"""
        if robot_id not in self.robot_state_machines:
            return
            
        state_machine = self.robot_state_machines[robot_id]
        
        if target_yaw is None:
            current_pose = self.get_pose_func_map[robot_id]()
            if current_pose:
                current_x, current_y, _ = current_pose
                target_yaw = atan2(target_y - current_y, target_x - current_x)
            else:
                target_yaw = 0.0
        
        state_machine.set_goal(target_x, target_y, target_yaw)
        
        # PID 컨트롤러 리셋
        self.linear_pid_controllers[robot_id].reset()
        self.angular_pid_controllers[robot_id].reset()
        
        self.get_logger().info(
            f"🎯 로봇{robot_id} PID 목표 설정: x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f}"
        )

    def set_robot_path(self, robot_id, waypoints):
        """로봇 경로 설정"""
        if robot_id not in self.robot_state_machines:
            return
            
        state_machine = self.robot_state_machines[robot_id]
        state_machine.set_path(waypoints)
        
        # PID 컨트롤러 리셋
        self.linear_pid_controllers[robot_id].reset()
        self.angular_pid_controllers[robot_id].reset()
        
        self.get_logger().info(f"🛤️ 로봇{robot_id} PID 경로 설정: {len(waypoints)}개 웨이포인트")

    def stop_robot(self, robot_id):
        """로봇 정지"""
        if robot_id not in self.robot_state_machines:
            return
            
        state_machine = self.robot_state_machines[robot_id]
        state_machine.stop()
        
        # 정지 명령 전송
        cmd = Twist()
        self.cmd_publishers[robot_id].publish(cmd)
        
        # PID 컨트롤러 리셋
        self.linear_pid_controllers[robot_id].reset()
        self.angular_pid_controllers[robot_id].reset()
        
        self.get_logger().info(f"🛑 로봇{robot_id} PID 정지")

    def control_robot_pid(self, robot_id):
        """PID를 이용한 로봇 제어"""
        if robot_id not in self.robot_state_machines:
            self.get_logger().warn(f"🚫 로봇{robot_id} 상태 머신이 존재하지 않음")
            return
            
        state_machine = self.robot_state_machines[robot_id]
        
        if state_machine.state == 'IDLE' or state_machine.goal is None:
            # self.get_logger().debug(f"🔍 로봇{robot_id} 상태: {state_machine.state}, 목표: {state_machine.goal}")
            return
            
        current_pose = self.get_pose_func_map[robot_id]()
        if not current_pose:
            self.get_logger().warn(f"⚠️ 로봇{robot_id} 포즈 데이터 없음")
            return
            
        current_x, current_y, current_yaw = current_pose
        target_x, target_y, target_yaw = state_machine.goal
        
        # 상태 머신 위치 업데이트
        state_machine.update_position((current_x, current_y, current_yaw))
        
        # Stuck 확인
        if state_machine.is_stuck():
            self.get_logger().warn(f"⚠️ 로봇{robot_id} Stuck 감지! 목표 재설정 필요")
            # Stuck 해제를 위한 작은 움직임 또는 경로 재계획 가능
            
        # 목표까지의 거리와 각도
        dx = target_x - current_x
        dy = target_y - current_y
        distance = sqrt(dx*dx + dy*dy)
        target_angle = atan2(dy, dx)
        
        cmd = Twist()
        
        # 위치 도달 확인
        if distance < self.config['position_tolerance']:
            # 다음 웨이포인트 확인
            if state_machine.next_waypoint():
                self.get_logger().info(f"📍 로봇{robot_id} 웨이포인트 {state_machine.current_path_index} 도달")
                return
            else:
                # 최종 목표 도달 - 각도 조정
                angle_error = self.normalize_angle(target_yaw - current_yaw)
                if abs(angle_error) > self.config['angle_tolerance']:
                    angular_output = self.angular_pid_controllers[robot_id].update(angle_error)
                    cmd.angular.z = angular_output
                    state_machine.state = 'ROTATING'
                else:
                    # 완전 도달
                    state_machine.state = 'ARRIVED'
                    self.stop_robot(robot_id)
                    self.get_logger().info(f"✅ 로봇{robot_id} PID 제어로 목표 도달!")
                    return
        else:
            # 목표로 이동
            angle_error = self.normalize_angle(target_angle - current_yaw)
            
            # 각속도 PID 제어
            angular_output = self.angular_pid_controllers[robot_id].update(angle_error)
            
            # 선속도 PID 제어 (거리 기반)
            # 목표 근처에서 속도 감소
            speed_factor = 1.0
            if distance < self.config['approach_distance']:
                speed_factor = distance / self.config['approach_distance']
                speed_factor = max(0.1, speed_factor)  # 최소 10% 속도 유지
            
            linear_output = self.linear_pid_controllers[robot_id].update(distance)
            linear_output *= speed_factor
            
            # 각도 오차가 클 때 선속도 감소
            if abs(angle_error) > 0.5:  # 약 30도
                linear_output *= 0.3
            elif abs(angle_error) > 0.3:  # 약 17도  
                linear_output *= 0.6
                
            cmd.linear.x = linear_output
            cmd.angular.z = angular_output
            
            state_machine.state = 'MOVING'
        
        # cmd 발행 디버깅
        if cmd.linear.x != 0.0 or cmd.angular.z != 0.0:
            self.get_logger().info(f"📤 로봇{robot_id} CMD 발행: linear={cmd.linear.x:.3f}, angular={cmd.angular.z:.3f}")
        
        self.cmd_publishers[robot_id].publish(cmd)

    def normalize_angle(self, angle):
        """각도 정규화"""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def publish_pose(self, robot_id):
        """포즈 퍼블리시"""
        pose_func = self.get_pose_func_map[robot_id]
        pose = pose_func()
        if pose is None:
            return

        x, y, yaw = pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.orientation.w = cos(yaw / 2.0)

        self.pose_publishers[robot_id].publish(msg)
        
    def reload_config(self, config_path="pid_config.yaml"):
        """PID 설정 다시 로드"""
        self.config = load_pid_config(config_path)
        
        # 모든 PID 컨트롤러 업데이트
        for robot_id in self.linear_pid_controllers:
            linear_config = self.config['linear_pid']
            angular_config = self.config['angular_pid']
            
            # 기존 컨트롤러 상태는 유지하고 파라미터만 업데이트
            self.linear_pid_controllers[robot_id].kp = linear_config['kp']
            self.linear_pid_controllers[robot_id].ki = linear_config['ki']
            self.linear_pid_controllers[robot_id].kd = linear_config['kd']
            self.linear_pid_controllers[robot_id].max_output = self.config['max_linear_speed']
            self.linear_pid_controllers[robot_id].min_output = -self.config['max_linear_speed']
            
            self.angular_pid_controllers[robot_id].kp = angular_config['kp']
            self.angular_pid_controllers[robot_id].ki = angular_config['ki']
            self.angular_pid_controllers[robot_id].kd = angular_config['kd']
            self.angular_pid_controllers[robot_id].max_output = self.config['max_angular_speed']
            self.angular_pid_controllers[robot_id].min_output = -self.config['max_angular_speed']
            
        self.get_logger().info("🔄 PID 설정 다시 로드됨")

# === 웹캠 추적 스레드 (기존과 동일) ===
class WebcamThread(threading.Thread):
    def __init__(self, tag_id, camera_index=0, visualize=False, frame_callback=None):
        super().__init__()
        self.frame_callback = frame_callback
        self.tag_id = tag_id
        self.camera_index = camera_index
        self.visualize = visualize
        self.result_lock = threading.Lock()
        self.pose = None
        self.running = True
        self.p0 = None
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.printed_once = True
        self.grid_corners = None
        self.all_tag_poses = []

    def interpolate(self, p1, p2, t):
        return (int(p1[0] + t * (p2[0] - p1[0])), int(p1[1] + t * (p2[1] - p1[1])))

    def generate_grid_points(self, h_div, v_div):
        if self.grid_corners is None or len(self.grid_corners) != 4:
            return []

        lefts = [self.interpolate(self.p0, self.p2, i / h_div) for i in range(h_div + 1)]
        rights = [self.interpolate(self.p1, self.p3, i / h_div) for i in range(h_div + 1)]
        tops = [self.interpolate(self.p0, self.p1, i / v_div) for i in range(v_div + 1)]
        bottoms = [self.interpolate(self.p2, self.p3, i / v_div) for i in range(v_div + 1)]

        grid_points = []
        for row, (l, r) in enumerate(zip(lefts, rights)):
            for col, (t, b) in enumerate(zip(tops, bottoms)):
                pt = self.interpolate(l, r, col / v_div)
                grid_points.append((row, col, pt))

        return grid_points

    def visualize_grid(self, img, grid_points, rows, cols):
        for row in range(rows):
            pts = [pt for r, c, pt in grid_points if r == row]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (255, 255, 0), 2)

        for col in range(cols):
            pts = [pt for r, c, pt in grid_points if c == col]
            for i in range(len(pts) - 1):
                cv2.line(img, pts[i], pts[i + 1], (0, 255, 255), 2)

        for row, col, pt in grid_points:
            cv2.circle(img, pt, 2, (0, 0, 255), 2)

    def find_closest(self, center):
        min_dist = float("inf")
        closest = center
        for c in self.all_tag_poses:
            c_pt = c["pose"][:2]
            dist = np.linalg.norm(np.array(c_pt) - np.array(center))
            if dist < min_dist:
                min_dist = dist
                closest = c_pt
        return tuple(closest)

    def get_tag_pose(self, tag_id):
        with self.result_lock:
            return [entry["pose"] for entry in self.all_tag_poses if entry["id"] == tag_id]

    def get_all_tag_poses(self):
        with self.result_lock:
            return list(self.all_tag_poses)

    def draw_grid_corners(self, frame):
        if None in (self.p0, self.p1, self.p2, self.p3):
            self.p0 = self.find_closest(TOP_LEFT)
            self.p1 = self.find_closest(TOP_RIGHT)
            self.p2 = self.find_closest(BOT_LEFT)
            self.p3 = self.find_closest(BOT_RIGHT)

        self.grid_corners = [self.p0, self.p1, self.p2, self.p3]
        if self.grid_corners:
            grid_points = self.generate_grid_points(horizontal_divisions, vertical_divisions)
            self.visualize_grid(frame, grid_points, ROWS, COLS)

    def redraw_grid_corners(self, frame):
        self.p0 = self.find_closest(TOP_LEFT)
        self.p1 = self.find_closest(TOP_RIGHT)
        self.p2 = self.find_closest(BOT_LEFT)
        self.p3 = self.find_closest(BOT_RIGHT)

        self.grid_corners = [self.p0, self.p1, self.p2, self.p3]
        if self.grid_corners:
            grid_points = self.generate_grid_points(horizontal_divisions, vertical_divisions)
            self.visualize_grid(frame, grid_points, ROWS, COLS)

    def get_transed_pose_for_tag(self, tag_id):
        poses = self.get_tag_pose(tag_id)
        if not poses:
            return None
        x, y, yaw = poses[0]
        real_x, real_y = self.transed_point(x, y)
        return real_x, real_y, yaw

    def transed_point(self, x, y):
        if self.grid_corners is not None:
            grid_corners = np.array(self.grid_corners, dtype=np.float32)
        unit_square = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=np.float32)
        try:
            Minv = cv2.getPerspectiveTransform(grid_corners, unit_square)
        except cv2.error as e:
            print("⚠️ Perspective 변환 실패:", e)
            return None

        img_point = np.array([[[x, y]]], dtype=np.float32)
        relative_point = cv2.perspectiveTransform(img_point, Minv)
        u, v = relative_point[0][0]
        real_x = u * REAL_MAX_WIDTH
        real_y = REAL_MAX_HEIGHT - v * REAL_MAX_HEIGHT
        return real_x, real_y

    def run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)

        if not cap.isOpened():
            print("❌ 웹캠 열기 실패")
            return

        detector = Detector(families="tag36h11")
        print(f"🎯 AprilTag ID {self.tag_id} PID 추적 중...")

        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("오류: 프레임을 읽을 수 없습니다.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray, estimate_tag_pose=False)

            current_tag_poses_list = []
            for tag in tags:
                cX, cY = int(tag.center[0]), int(tag.center[1])
                pt0, pt1 = tag.corners[0], tag.corners[1]
                dx = pt1[0] - pt0[0]
                dy = pt1[1] - pt0[1]
                yaw = atan2(dy, dx)

                corners = np.array(tag.corners, dtype=np.int32)
                cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                arrow_len = 40
                arrow_tip = (int(cX + arrow_len * cos(yaw)), int(cY + arrow_len * sin(yaw)))
                cv2.arrowedLine(frame, (cX, cY), arrow_tip, (0, 0, 255), 3, tipLength=0.3)

                tag_id_text = f"ID: {tag.tag_id}"
                cv2.putText(frame, tag_id_text, (cX - 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(frame, tag_id_text, (cX - 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                current_tag_poses_list.append({
                    "id": tag.tag_id,
                    "pose": (cX, cY, -yaw),
                    "corners": [(int(p[0]), int(p[1])) for p in tag.corners],
                })

                if tag.tag_id == self.tag_id:
                    with self.result_lock:
                        self.pose = (cX, cY, -yaw)

            with self.result_lock:
                self.all_tag_poses = current_tag_poses_list

            if self.all_tag_poses:
                if self.visualize:
                    self.draw_grid_corners(frame)

            if self.frame_callback:
                self.frame_callback(frame)

        cap.release()
        cv2.destroyAllWindows()
        print("🛑 웹캠 스레드 종료")

    def stop(self):
        self.running = False


from PySide6.QtWidgets import QLabel
from PySide6.QtCore import Qt, Signal, QPoint

class ImageLabel(QLabel):
    mouse_clicked = Signal(QPoint)
    mouse_moved = Signal(QPoint)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)

    def mouseMoveEvent(self, event):
        self.mouse_moved.emit(event.position().toPoint())

    def mousePressEvent(self, event):
        self.mouse_clicked.emit(event.position().toPoint())


from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, 
    QSizePolicy, QListWidget, QListWidgetItem, QLineEdit, QCheckBox, QComboBox,
    QTextEdit, QScrollArea, QFrame, QSpinBox, QDoubleSpinBox, QTabWidget,
    QGroupBox, QGridLayout
)
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, Signal

class PIDTuningWidget(QWidget):
    """PID 튜닝 위젯"""
    def __init__(self, ros_node, config_path="pid_config.yaml"):
        super().__init__()
        self.ros_node = ros_node
        self.config_path = config_path
        self.setup_ui()
        self.load_current_config()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # 선속도 PID 그룹
        linear_group = QGroupBox("선속도 PID")
        linear_layout = QGridLayout()
        
        self.linear_kp = QDoubleSpinBox()
        self.linear_kp.setDecimals(3)
        self.linear_kp.setRange(0.0, 10.0)
        self.linear_kp.setSingleStep(0.1)
        
        self.linear_ki = QDoubleSpinBox()
        self.linear_ki.setDecimals(3)
        self.linear_ki.setRange(0.0, 10.0)
        self.linear_ki.setSingleStep(0.01)
        
        self.linear_kd = QDoubleSpinBox()
        self.linear_kd.setDecimals(3)
        self.linear_kd.setRange(0.0, 10.0)
        self.linear_kd.setSingleStep(0.01)
        
        linear_layout.addWidget(QLabel("Kp:"), 0, 0)
        linear_layout.addWidget(self.linear_kp, 0, 1)
        linear_layout.addWidget(QLabel("Ki:"), 1, 0)
        linear_layout.addWidget(self.linear_ki, 1, 1)
        linear_layout.addWidget(QLabel("Kd:"), 2, 0)
        linear_layout.addWidget(self.linear_kd, 2, 1)
        
        linear_group.setLayout(linear_layout)
        
        # 각속도 PID 그룹
        angular_group = QGroupBox("각속도 PID")
        angular_layout = QGridLayout()
        
        self.angular_kp = QDoubleSpinBox()
        self.angular_kp.setDecimals(3)
        self.angular_kp.setRange(0.0, 10.0)
        self.angular_kp.setSingleStep(0.1)
        
        self.angular_ki = QDoubleSpinBox()
        self.angular_ki.setDecimals(3)
        self.angular_ki.setRange(0.0, 10.0)
        self.angular_ki.setSingleStep(0.01)
        
        self.angular_kd = QDoubleSpinBox()
        self.angular_kd.setDecimals(3)
        self.angular_kd.setRange(0.0, 10.0)
        self.angular_kd.setSingleStep(0.01)
        
        angular_layout.addWidget(QLabel("Kp:"), 0, 0)
        angular_layout.addWidget(self.angular_kp, 0, 1)
        angular_layout.addWidget(QLabel("Ki:"), 1, 0)
        angular_layout.addWidget(self.angular_ki, 1, 1)
        angular_layout.addWidget(QLabel("Kd:"), 2, 0)
        angular_layout.addWidget(self.angular_kd, 2, 1)
        
        angular_group.setLayout(angular_layout)
        
        # 기타 설정
        settings_group = QGroupBox("기타 설정")
        settings_layout = QGridLayout()
        
        self.position_tolerance = QDoubleSpinBox()
        self.position_tolerance.setDecimals(3)
        self.position_tolerance.setRange(0.001, 1.0)
        self.position_tolerance.setSingleStep(0.005)
        
        self.angle_tolerance = QDoubleSpinBox()
        self.angle_tolerance.setDecimals(3)
        self.angle_tolerance.setRange(0.001, 1.0)
        self.angle_tolerance.setSingleStep(0.01)
        
        self.max_linear_speed = QDoubleSpinBox()
        self.max_linear_speed.setDecimals(2)
        self.max_linear_speed.setRange(0.1, 2.0)
        self.max_linear_speed.setSingleStep(0.1)
        
        self.max_angular_speed = QDoubleSpinBox()
        self.max_angular_speed.setDecimals(2)
        self.max_angular_speed.setRange(0.1, 5.0)
        self.max_angular_speed.setSingleStep(0.1)
        
        settings_layout.addWidget(QLabel("위치 허용오차 (m):"), 0, 0)
        settings_layout.addWidget(self.position_tolerance, 0, 1)
        settings_layout.addWidget(QLabel("각도 허용오차 (rad):"), 1, 0)
        settings_layout.addWidget(self.angle_tolerance, 1, 1)
        settings_layout.addWidget(QLabel("최대 선속도 (m/s):"), 2, 0)
        settings_layout.addWidget(self.max_linear_speed, 2, 1)
        settings_layout.addWidget(QLabel("최대 각속도 (rad/s):"), 3, 0)
        settings_layout.addWidget(self.max_angular_speed, 3, 1)
        
        settings_group.setLayout(settings_layout)
        
        # 버튼들
        button_layout = QHBoxLayout()
        
        self.apply_btn = QPushButton("적용")
        self.apply_btn.clicked.connect(self.apply_settings)
        
        self.save_btn = QPushButton("저장")
        self.save_btn.clicked.connect(self.save_settings)
        
        self.load_btn = QPushButton("다시 로드")
        self.load_btn.clicked.connect(self.load_current_config)
        
        button_layout.addWidget(self.apply_btn)
        button_layout.addWidget(self.save_btn)
        button_layout.addWidget(self.load_btn)
        
        # 레이아웃 구성
        layout.addWidget(linear_group)
        layout.addWidget(angular_group)
        layout.addWidget(settings_group)
        layout.addLayout(button_layout)
        layout.addStretch()
        
        self.setLayout(layout)
    
    def load_current_config(self):
        """현재 설정 로드"""
        config = load_pid_config(self.config_path)
        
        # 선속도 PID
        self.linear_kp.setValue(config['linear_pid']['kp'])
        self.linear_ki.setValue(config['linear_pid']['ki'])
        self.linear_kd.setValue(config['linear_pid']['kd'])
        
        # 각속도 PID
        self.angular_kp.setValue(config['angular_pid']['kp'])
        self.angular_ki.setValue(config['angular_pid']['ki'])
        self.angular_kd.setValue(config['angular_pid']['kd'])
        
        # 기타 설정
        self.position_tolerance.setValue(config['position_tolerance'])
        self.angle_tolerance.setValue(config['angle_tolerance'])
        self.max_linear_speed.setValue(config['max_linear_speed'])
        self.max_angular_speed.setValue(config['max_angular_speed'])
        
        print("🔄 PID 설정 UI에 로드됨")
    
    def apply_settings(self):
        """설정 적용"""
        if self.ros_node:
            # 메모리 상의 설정 업데이트
            self.ros_node.config['linear_pid']['kp'] = self.linear_kp.value()
            self.ros_node.config['linear_pid']['ki'] = self.linear_ki.value()
            self.ros_node.config['linear_pid']['kd'] = self.linear_kd.value()
            
            self.ros_node.config['angular_pid']['kp'] = self.angular_kp.value()
            self.ros_node.config['angular_pid']['ki'] = self.angular_ki.value()
            self.ros_node.config['angular_pid']['kd'] = self.angular_kd.value()
            
            self.ros_node.config['position_tolerance'] = self.position_tolerance.value()
            self.ros_node.config['angle_tolerance'] = self.angle_tolerance.value()
            self.ros_node.config['max_linear_speed'] = self.max_linear_speed.value()
            self.ros_node.config['max_angular_speed'] = self.max_angular_speed.value()
            
            # PID 컨트롤러 업데이트
            for robot_id in self.ros_node.linear_pid_controllers:
                # 선속도 PID 업데이트
                self.ros_node.linear_pid_controllers[robot_id].kp = self.linear_kp.value()
                self.ros_node.linear_pid_controllers[robot_id].ki = self.linear_ki.value()
                self.ros_node.linear_pid_controllers[robot_id].kd = self.linear_kd.value()
                self.ros_node.linear_pid_controllers[robot_id].max_output = self.max_linear_speed.value()
                self.ros_node.linear_pid_controllers[robot_id].min_output = -self.max_linear_speed.value()
                
                # 각속도 PID 업데이트
                self.ros_node.angular_pid_controllers[robot_id].kp = self.angular_kp.value()
                self.ros_node.angular_pid_controllers[robot_id].ki = self.angular_ki.value()
                self.ros_node.angular_pid_controllers[robot_id].kd = self.angular_kd.value()
                self.ros_node.angular_pid_controllers[robot_id].max_output = self.max_angular_speed.value()
                self.ros_node.angular_pid_controllers[robot_id].min_output = -self.max_angular_speed.value()
            
            print("✅ PID 설정 실시간 적용됨")
    
    def save_settings(self):
        """설정 파일에 저장"""
        config = {
            'linear_pid': {
                'kp': self.linear_kp.value(),
                'ki': self.linear_ki.value(),
                'kd': self.linear_kd.value()
            },
            'angular_pid': {
                'kp': self.angular_kp.value(),
                'ki': self.angular_ki.value(),
                'kd': self.angular_kd.value()
            },
            'position_tolerance': self.position_tolerance.value(),
            'angle_tolerance': self.angle_tolerance.value(),
            'max_linear_speed': self.max_linear_speed.value(),
            'max_angular_speed': self.max_angular_speed.value(),
            'stuck_threshold': 3.0,
            'approach_distance': 0.2
        }
        
        try:
            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            print(f"💾 PID 설정 파일 저장됨: {self.config_path}")
            self.apply_settings()  # 저장 후 적용
        except Exception as e:
            print(f"❌ PID 설정 저장 실패: {e}")

class ImageWindow(QWidget):
    def __init__(self, robot_tag_map):
        super().__init__()
        self.setWindowTitle("AprilTag PID Robot Controller")
        self.setGeometry(100, 100, 1600, 900)

        self.robot_tag_map = robot_tag_map
        self.selected_robot_id = None
        self.cursor_pos = None
        self.webcam_thread = None
        self.current_frame = None
        self.ros_node = None
        
        # 경로 계획 상태
        self.waypoints = {}
        self.start_points = {}
        self.goal_points = {}
        self.path_planning_mode = False
        self.current_waypoint_robot = None

        self.setup_ui()

    def setup_ui(self):
        # 탭 위젯 생성
        tab_widget = QTabWidget()
        
        # 메인 제어 탭
        main_tab = QWidget()
        self.setup_main_tab(main_tab)
        tab_widget.addTab(main_tab, "🎮 로봇 제어")
        
        # PID 튜닝 탭 (나중에 ROS 노드 설정 후 생성)
        self.pid_tab = None
        tab_widget.addTab(QWidget(), "⚙️ PID 튜닝")
        
        # 전체 레이아웃
        main_layout = QHBoxLayout()
        
        # 왼쪽: 탭 위젯
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        left_layout.addWidget(tab_widget)
        left_widget.setLayout(left_layout)
        left_widget.setFixedWidth(400)
        
        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.image_label, stretch=1)
        self.setLayout(main_layout)
        
        # 탭 위젯 참조 저장
        self.tab_widget = tab_widget

    def setup_main_tab(self, main_tab):
        # 로봇 리스트
        self.robot_list = QListWidget()
        self.robot_list.setFixedHeight(150)
        self.robot_list.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        self.robot_list.itemClicked.connect(self.handle_robot_select)

        for robot_id in self.robot_tag_map:
            item = QListWidgetItem(f"🤖 로봇-{robot_id}")
            self.robot_list.addItem(item)

        # 태그 정보 폼
        self.tag_info_label = QLabel("태그 ID:")
        self.tag_edit = QLineEdit()
        self.tag_edit.setFixedWidth(100)
        self.save_button = QPushButton("💾 저장")
        self.save_button.setFixedWidth(100)
        self.save_button.clicked.connect(self.save_tag_id)
        
        self.tag_x_label = QLabel("X:")
        self.tag_x = QLineEdit()
        self.tag_x.setFixedWidth(80)
        self.tag_y_label = QLabel("Y:")
        self.tag_y = QLineEdit()
        self.tag_y.setFixedWidth(80)
        self.tag_yaw_label = QLabel("Yaw:")
        self.tag_yaw = QLineEdit()
        self.tag_yaw.setFixedWidth(80)

        # 로봇 상태 표시
        self.status_label = QLabel("로봇 상태:")
        self.status_text = QLabel("IDLE")
        self.status_text.setStyleSheet("font-weight: bold; color: blue;")

        # 로봇 제어 섹션
        control_frame = QFrame()
        control_frame.setFrameStyle(QFrame.Shape.Box)
        
        self.control_label = QLabel("🎮 PID 로봇 제어")
        self.control_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        
        # 제어 모드 선택
        self.mode_label = QLabel("제어 모드:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["단일 목표", "경로 계획", "수동 제어"])
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        
        # 목표 설정 버튼들
        self.set_start_btn = QPushButton("🎯 시작점 설정")
        self.set_start_btn.clicked.connect(self.set_start_point_mode)
        self.set_start_btn.setEnabled(False)
        
        self.set_goal_btn = QPushButton("🏁 목표점 설정")
        self.set_goal_btn.clicked.connect(self.set_goal_point_mode)
        self.set_goal_btn.setEnabled(False)
        
        self.add_waypoint_btn = QPushButton("📍 웨이포인트 추가")
        self.add_waypoint_btn.clicked.connect(self.add_waypoint_mode)
        self.add_waypoint_btn.setEnabled(False)
        
        self.execute_btn = QPushButton("▶️ 실행")
        self.execute_btn.clicked.connect(self.execute_path)
        self.execute_btn.setEnabled(False)
        
        self.stop_btn = QPushButton("⏹️ 정지")
        self.stop_btn.clicked.connect(self.stop_robot)
        self.stop_btn.setEnabled(False)
        
        self.clear_path_btn = QPushButton("🗑️ 경로 지우기")
        self.clear_path_btn.clicked.connect(self.clear_path)

        # 경로 정보 표시
        self.path_info = QTextEdit()
        self.path_info.setMaximumHeight(100)
        self.path_info.setReadOnly(True)

        # 리셋 버튼
        self.refresh_button = QPushButton("🔄 그리드 리셋")
        self.refresh_button.clicked.connect(self.handle_refresh_click)

        # 이미지 레이블
        self.image_label = ImageLabel(self)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        size_policy = QSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.image_label.setSizePolicy(size_policy)
        self.image_label.mouse_moved.connect(self.handle_mouse_move)
        self.image_label.mouse_clicked.connect(self.handle_mouse_click)

        # 레이아웃 구성
        tag_form = QVBoxLayout()
        tag_form.addWidget(self.tag_info_label)
        tag_form.addWidget(self.tag_edit)
        tag_form.addWidget(self.save_button)

        pos_layout = QHBoxLayout()
        pos_layout.addWidget(self.tag_x_label)
        pos_layout.addWidget(self.tag_x)
        pos_layout.addWidget(self.tag_y_label)
        pos_layout.addWidget(self.tag_y)

        yaw_layout = QHBoxLayout()
        yaw_layout.addWidget(self.tag_yaw_label)
        yaw_layout.addWidget(self.tag_yaw)

        status_layout = QHBoxLayout()
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.status_text)

        control_layout = QVBoxLayout()
        control_layout.addWidget(self.control_label)
        control_layout.addWidget(self.mode_label)
        control_layout.addWidget(self.mode_combo)
        control_layout.addWidget(self.set_start_btn)
        control_layout.addWidget(self.set_goal_btn)
        control_layout.addWidget(self.add_waypoint_btn)
        control_layout.addWidget(self.execute_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addWidget(self.clear_path_btn)
        control_layout.addWidget(QLabel("경로 정보:"))
        control_layout.addWidget(self.path_info)
        
        control_frame.setLayout(control_layout)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.robot_list)
        main_layout.addLayout(tag_form)
        main_layout.addLayout(pos_layout)
        main_layout.addLayout(yaw_layout)
        main_layout.addLayout(status_layout)
        main_layout.addWidget(control_frame)
        main_layout.addWidget(self.refresh_button)
        main_layout.addStretch()

        main_tab.setLayout(main_layout)

    def set_ros_node(self, node):
        """ROS 노드 설정 및 PID 튜닝 탭 생성"""
        self.ros_node = node
        
        # PID 튜닝 탭 생성
        if self.pid_tab is None:
            self.pid_tab = PIDTuningWidget(self.ros_node)
            self.tab_widget.removeTab(1)  # 빈 탭 제거
            self.tab_widget.addTab(self.pid_tab, "⚙️ PID 튜닝")
        
        # 상태 업데이트 타이머
        self.create_status_timer()

    def create_status_timer(self):
        """로봇 상태 업데이트 타이머"""
        from PySide6.QtCore import QTimer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_robot_status)
        self.status_timer.start(200)  # 200ms마다 업데이트

    def update_robot_status(self):
        """로봇 상태 업데이트"""
        if self.selected_robot_id is not None and self.ros_node is not None:
            robot_id = self.selected_robot_id
            if robot_id in self.ros_node.robot_state_machines:
                state_machine = self.ros_node.robot_state_machines[robot_id]
                state = state_machine.state
                
                # 상태에 따른 색상 변경
                if state == 'IDLE':
                    self.status_text.setStyleSheet("font-weight: bold; color: blue;")
                elif state == 'MOVING':
                    self.status_text.setStyleSheet("font-weight: bold; color: green;")
                elif state == 'ROTATING':
                    self.status_text.setStyleSheet("font-weight: bold; color: orange;")
                elif state == 'ARRIVED':
                    self.status_text.setStyleSheet("font-weight: bold; color: purple;")
                else:
                    self.status_text.setStyleSheet("font-weight: bold; color: red;")
                
                # Stuck 상태 표시
                if state_machine.is_stuck():
                    self.status_text.setText(f"{state} (STUCK)")
                    self.status_text.setStyleSheet("font-weight: bold; color: red;")
                else:
                    self.status_text.setText(state)

    def on_mode_changed(self, mode):
        """제어 모드 변경 시 UI 업데이트"""
        if mode == "단일 목표":
            self.set_start_btn.setEnabled(False)
            self.set_goal_btn.setEnabled(self.selected_robot_id is not None)
            self.add_waypoint_btn.setEnabled(False)
        elif mode == "경로 계획":
            self.set_start_btn.setEnabled(self.selected_robot_id is not None)
            self.set_goal_btn.setEnabled(self.selected_robot_id is not None)
            self.add_waypoint_btn.setEnabled(self.selected_robot_id is not None)
        else:  # 수동 제어
            self.set_start_btn.setEnabled(False)
            self.set_goal_btn.setEnabled(False)
            self.add_waypoint_btn.setEnabled(False)
        
        self.path_planning_mode = False
        self.current_waypoint_robot = None

    def set_start_point_mode(self):
        """시작점 설정 모드"""
        self.path_planning_mode = "start"
        self.set_start_btn.setStyleSheet("background-color: #4CAF50;")
        self.set_goal_btn.setStyleSheet("")
        self.add_waypoint_btn.setStyleSheet("")

    def set_goal_point_mode(self):
        """목표점 설정 모드"""
        self.path_planning_mode = "goal"
        self.set_start_btn.setStyleSheet("")
        self.set_goal_btn.setStyleSheet("background-color: #f44336;")
        self.add_waypoint_btn.setStyleSheet("")

    def add_waypoint_mode(self):
        """웨이포인트 추가 모드"""
        self.path_planning_mode = "waypoint"
        self.set_start_btn.setStyleSheet("")
        self.set_goal_btn.setStyleSheet("")
        self.add_waypoint_btn.setStyleSheet("background-color: #2196F3;")

    def clear_path(self):
        """경로 지우기"""
        if self.selected_robot_id is not None:
            self.waypoints[self.selected_robot_id] = []
            self.start_points.pop(self.selected_robot_id, None)
            self.goal_points.pop(self.selected_robot_id, None)
            self.update_path_info()
            print(f"🗑️ 로봇{self.selected_robot_id} 경로 지워짐")

    def execute_path(self):
        """경로 실행"""
        if self.selected_robot_id is None or self.ros_node is None:
            return
            
        mode = self.mode_combo.currentText()
        robot_id = self.selected_robot_id
        
        if mode == "단일 목표":
            if robot_id in self.goal_points:
                goal_x, goal_y = self.goal_points[robot_id]
                self.ros_node.set_robot_goal(robot_id, goal_x, goal_y)
                print(f"▶️ 로봇{robot_id} PID 단일 목표로 이동: ({goal_x:.3f}, {goal_y:.3f})")
            else:
                print(f"⚠️ 로봇{robot_id}의 목표점이 설정되지 않았습니다.")
        
        elif mode == "경로 계획":
            if robot_id in self.waypoints and self.waypoints[robot_id]:
                path = self.waypoints[robot_id].copy()
                if robot_id in self.goal_points:
                    path.append(self.goal_points[robot_id])
                self.ros_node.set_robot_path(robot_id, path)
                print(f"▶️ 로봇{robot_id} PID 경로 실행: {len(path)}개 웨이포인트")
            else:
                print(f"⚠️ 로봇{robot_id}의 경로가 설정되지 않았습니다.")
        
        self.execute_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

    def stop_robot(self):
        """로봇 정지"""
        if self.selected_robot_id is not None and self.ros_node is not None:
            self.ros_node.stop_robot(self.selected_robot_id)
            self.execute_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)

    def update_path_info(self):
        """경로 정보 업데이트"""
        if self.selected_robot_id is None:
            self.path_info.clear()
            return
            
        robot_id = self.selected_robot_id
        info_text = f"로봇 {robot_id} 경로 정보:\n"
        
        if robot_id in self.start_points:
            x, y = self.start_points[robot_id]
            info_text += f"🎯 시작점: ({x:.3f}, {y:.3f})\n"
        
        if robot_id in self.waypoints and self.waypoints[robot_id]:
            info_text += f"📍 웨이포인트 ({len(self.waypoints[robot_id])}개):\n"
            for i, (x, y) in enumerate(self.waypoints[robot_id]):
                info_text += f"  {i+1}. ({x:.3f}, {y:.3f})\n"
        
        if robot_id in self.goal_points:
            x, y = self.goal_points[robot_id]
            info_text += f"🏁 목표점: ({x:.3f}, {y:.3f})\n"
        
        self.path_info.setText(info_text)
        
        # 실행 버튼 활성화 조건
        mode = self.mode_combo.currentText()
        if mode == "단일 목표":
            self.execute_btn.setEnabled(robot_id in self.goal_points)
        elif mode == "경로 계획":
            has_waypoints = robot_id in self.waypoints and len(self.waypoints[robot_id]) > 0
            has_goal = robot_id in self.goal_points
            self.execute_btn.setEnabled(has_waypoints or has_goal)

    def handle_mouse_click(self):
        """마우스 클릭 처리"""
        if self.webcam_thread is None or self.webcam_thread.grid_corners is None:
            print("⚠️ 그리드 정보 없음")
            return

        if self.cursor_pos is None:
            print("⚠️ 커서 위치 정보 없음")
            return

        x, y = self.cursor_pos[0], self.cursor_pos[1]
        real_x, real_y = self.webcam_thread.transed_point(x, y)

        if not (0 <= real_x <= REAL_MAX_WIDTH and 0 <= real_y <= REAL_MAX_HEIGHT):
            print("⚠️ 클릭 위치가 그리드 범위를 벗어남")
            return

        print(f"🖱️ 클릭 위치: 이미지({x}, {y}) → 실제({real_x:.3f}, {real_y:.3f})")

        if self.selected_robot_id is None:
            print("⚠️ 로봇을 먼저 선택하세요")
            return

        robot_id = self.selected_robot_id
        
        # 제어 모드에 따른 처리
        if self.path_planning_mode == "start":
            self.start_points[robot_id] = (real_x, real_y)
            print(f"🎯 로봇{robot_id} 시작점 설정: ({real_x:.3f}, {real_y:.3f})")
            self.path_planning_mode = False
            self.set_start_btn.setStyleSheet("")
            
        elif self.path_planning_mode == "goal":
            self.goal_points[robot_id] = (real_x, real_y)
            print(f"🏁 로봇{robot_id} 목표점 설정: ({real_x:.3f}, {real_y:.3f})")
            self.path_planning_mode = False
            self.set_goal_btn.setStyleSheet("")
            
        elif self.path_planning_mode == "waypoint":
            if robot_id not in self.waypoints:
                self.waypoints[robot_id] = []
            self.waypoints[robot_id].append((real_x, real_y))
            print(f"📍 로봇{robot_id} 웨이포인트 추가: ({real_x:.3f}, {real_y:.3f})")
            
        else:
            # 기본 모드: 즉시 목표점으로 이동
            mode = self.mode_combo.currentText()
            if mode == "단일 목표":
                self.goal_points[robot_id] = (real_x, real_y)
                if self.ros_node:
                    self.ros_node.set_robot_goal(robot_id, real_x, real_y)
                    print(f"🎯 로봇{robot_id} PID로 즉시 이동: ({real_x:.3f}, {real_y:.3f})")

        self.update_path_info()

    def handle_robot_select(self, item):
        """로봇 선택 처리"""
        text = item.text()
        if "로봇-" in text:
            robot_id = int(text.split("-")[1])
            self.selected_robot_id = robot_id
            
            # 기존 스타일 초기화
            for i in range(self.robot_list.count()):
                self.robot_list.item(i).setBackground(Qt.GlobalColor.white)
            
            # 선택된 항목 하이라이트
            item.setBackground(Qt.GlobalColor.cyan)

            # 태그 정보 업데이트
            tag_id = self.robot_tag_map.get(robot_id, "")
            self.tag_edit.setText(str(tag_id))
            
            if self.webcam_thread:
                poses = self.webcam_thread.get_tag_pose(tag_id)
                if poses:
                    pose = poses[0]
                    self.tag_x.setText(f"{pose[0]:.1f}")
                    self.tag_y.setText(f"{pose[1]:.1f}")
                    self.tag_yaw.setText(f"{pose[2]:.3f}")
            
            # 버튼 상태 업데이트
            mode = self.mode_combo.currentText()
            self.on_mode_changed(mode)
            self.update_path_info()
            
            print(f"🤖 로봇 {robot_id} 선택됨")

    def save_tag_id(self):
        """태그 ID 저장"""
        if self.selected_robot_id is not None:
            new_id = self.tag_edit.text()
            if new_id.isdigit():
                self.robot_tag_map[self.selected_robot_id] = int(new_id)
                print(f"💾 로봇{self.selected_robot_id} → 태그 ID: {new_id} 저장됨")

    def handle_mouse_move(self, pos: QPoint):
        """마우스 이동 처리"""
        if self.current_frame is None:
            return

        # QLabel 크기
        label_w = self.image_label.width()
        label_h = self.image_label.height()

        # 이미지 크기
        img_h, img_w, _ = self.current_frame.shape

        # 이미지와 라벨 사이즈의 비율 계산
        scale_w = label_w / img_w
        scale_h = label_h / img_h

        # KeepAspectRatio에 따라 실제 스케일을 선택
        scale = min(scale_w, scale_h)

        # 이미지가 QLabel 내에서 차지하는 실제 크기
        displayed_w = int(img_w * scale)
        displayed_h = int(img_h * scale)

        # QLabel 중앙 정렬 오프셋 계산
        offset_x = (label_w - displayed_w) // 2
        offset_y = (label_h - displayed_h) // 2

        # 마우스 위치 변환
        x_in_label = pos.x() - offset_x
        y_in_label = pos.y() - offset_y

        if x_in_label < 0 or y_in_label < 0 or x_in_label >= displayed_w or y_in_label >= displayed_h:
            self.cursor_pos = None
        else:
            real_x = int(x_in_label / scale)
            real_y = int(y_in_label / scale)
            self.cursor_pos = (real_x, real_y)

        self.refresh_image()

    def handle_refresh_click(self):
        """그리드 리셋"""
        if self.webcam_thread is not None and self.current_frame is not None:
            self.webcam_thread.redraw_grid_corners(self.current_frame)
            self.refresh_image()

    def set_webcam_thread(self, thread):
        """웹캠 스레드 설정"""
        self.webcam_thread = thread

    def update_image(self, frame):
        """이미지 업데이트"""
        self.current_frame = frame
        self.refresh_image()

    def resizeEvent(self, event):
        """창 크기 변경 시"""
        self.refresh_image()
        return super().resizeEvent(event)

    def refresh_image(self):
        """이미지 갱신"""
        if self.current_frame is None:
            return

        rgb_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)

        # 커서 위치 표시
        if self.cursor_pos is not None:
            x, y = self.cursor_pos
            if self.webcam_thread and self.webcam_thread.grid_corners:
                real_x, real_y = self.webcam_thread.transed_point(x, y)
                if 0 <= real_x <= REAL_MAX_WIDTH and 0 <= real_y <= REAL_MAX_HEIGHT:
                    cv2.putText(
                        rgb_image,
                        f"({real_x:.3f}, {real_y:.3f})",
                        (x + 10, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )
            cv2.circle(rgb_image, (x, y), 5, (255, 255, 255), 2)

        # 경로 시각화
        self.draw_paths(rgb_image)

        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

        pixmap = QPixmap.fromImage(q_image)
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.image_label.setPixmap(scaled)

    def draw_paths(self, image):
        """경로 시각화"""
        if not self.webcam_thread or not self.webcam_thread.grid_corners:
            return

        for robot_id in self.robot_tag_map.keys():
            # 시작점
            if robot_id in self.start_points:
                real_x, real_y = self.start_points[robot_id]
                img_pt = self.real_to_image_point(real_x, real_y)
                if img_pt:
                    cv2.circle(image, img_pt, 10, (0, 255, 0), 3)
                    cv2.putText(image, f"S{robot_id}", (img_pt[0]-10, img_pt[1]-15), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 웨이포인트들
            if robot_id in self.waypoints:
                prev_point = None
                for i, (real_x, real_y) in enumerate(self.waypoints[robot_id]):
                    img_pt = self.real_to_image_point(real_x, real_y)
                    if img_pt:
                        cv2.circle(image, img_pt, 8, (255, 0, 0), 2)
                        cv2.putText(image, f"{i+1}", (img_pt[0]-5, img_pt[1]+5), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        
                        if prev_point:
                            cv2.line(image, prev_point, img_pt, (255, 0, 0), 2)
                        prev_point = img_pt

            # 목표점
            if robot_id in self.goal_points:
                real_x, real_y = self.goal_points[robot_id]
                img_pt = self.real_to_image_point(real_x, real_y)
                if img_pt:
                    cv2.circle(image, img_pt, 12, (0, 0, 255), 3)
                    cv2.putText(image, f"G{robot_id}", (img_pt[0]-10, img_pt[1]-15), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    if robot_id in self.waypoints and self.waypoints[robot_id]:
                        last_waypoint = self.waypoints[robot_id][-1]
                        last_img_pt = self.real_to_image_point(last_waypoint[0], last_waypoint[1])
                        if last_img_pt:
                            cv2.line(image, last_img_pt, img_pt, (255, 0, 0), 2)

    def real_to_image_point(self, real_x, real_y):
        """실제 좌표를 이미지 좌표로 변환"""
        if not self.webcam_thread or not self.webcam_thread.grid_corners:
            return None
            
        try:
            u = real_x / REAL_MAX_WIDTH
            v = (REAL_MAX_HEIGHT - real_y) / REAL_MAX_HEIGHT
            
            grid_corners = np.array(self.webcam_thread.grid_corners, dtype=np.float32)
            unit_square = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=np.float32)
            
            M = cv2.getPerspectiveTransform(unit_square, grid_corners)
            
            norm_point = np.array([[[u, v]]], dtype=np.float32)
            img_point = cv2.perspectiveTransform(norm_point, M)
            
            x, y = int(img_point[0][0][0]), int(img_point[0][0][1])
            return (x, y)
            
        except Exception as e:
            print(f"⚠️ 좌표 변환 실패: {e}")
            return None


# === 메인 함수 ===
def main():
    # 로봇 ID와 에이프릴태그 ID 매핑
    robot_tag_map = {
        1: 2,  # 로봇 1 → 태그 2
        2: 4,  # 로봇 2 → 태그 4
        # 필요시 추가
    }

    app = QApplication(sys.argv)
    image_window = ImageWindow(robot_tag_map=robot_tag_map)
    image_window.show()

    # 웹캠 스레드 시작
    webcam_thread = WebcamThread(
        tag_id=TAG_ID,
        camera_index=CAMERA_INDEX,
        visualize=True,
        frame_callback=image_window.update_image,
    )
    webcam_thread.start()
    image_window.set_webcam_thread(webcam_thread)

    # ROS 초기화
    rclpy.init(args=None)
    robot_ids = list(robot_tag_map.keys())

    # pose 함수 매핑 생성
    def make_pose_func(tag_id):
        return lambda: webcam_thread.get_transed_pose_for_tag(tag_id)

    get_pose_func_map = {rid: make_pose_func(robot_tag_map[rid]) for rid in robot_ids}

    # ROS 노드 생성 (PID 적용)
    multi_publisher_node = PIDPoseMultiPublisher(
        robot_ids, get_pose_func_map, publish_period=0.1
    )
    image_window.set_ros_node(multi_publisher_node)

    # ROS 스핀 스레드
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(multi_publisher_node), daemon=True
    )
    ros_thread.start()

    print("🚀 AprilTag PID Robot Controller 시작!")
    print("📋 사용법:")
    print("  1. 로봇을 선택하세요")
    print("  2. 제어 모드를 선택하세요")
    print("  3. 시작점/목표점을 클릭으로 설정하세요")
    print("  4. PID 튜닝 탭에서 제어 파라미터를 조정하세요")
    print("  5. '실행' 버튼을 눌러 로봇을 움직이세요")

    # Qt 메인 루프 실행
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("🛑 종료 요청 받음")
    finally:
        multi_publisher_node.destroy_node()
        rclpy.shutdown()
        webcam_thread.stop()
        webcam_thread.join()
        print("✅ 프로그램 종료")

if __name__ == '__main__':
    main()