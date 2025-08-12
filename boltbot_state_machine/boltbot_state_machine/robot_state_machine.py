#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
이 스크립트는 로봇을 특정 목표 지점(x, y, yaw)으로 이동시키기 위한 상태 머신 기반의 제어기입니다.
로봇의 현재 위치는 `/camera_pose` 토픽에서, 목표 위치는 `/target_pose` 토픽에서 받아옵니다.
계산된 속도 명령은 `/cmd_vel` 토픽으로 발행됩니다.

주요 기능:
- 상태 기반 제어: 로봇의 행동을 '목표 방향으로 회전', '목표 지점으로 이동', '최종 각도로 회전', '목표 도달', '장애물 회피'의 5가지 상태로 나누어 관리합니다.
- PID 제어: 선속도와 각속도를 제어하기 위해 PID(Proportional-Integral-Derivative) 제어기를 사용합니다.
- YAML 기반 동적 파라미터 설정: 'pid_config.yaml' 파일에서 PID 게인 값, 허용 오차 등을 실시간으로 읽어와 반영합니다.
- 상태 모니터링: 상세 상태(/state)와 관제용 임무 상태(/task_status)를 발행합니다.
- 후진 지원: 목표가 로봇 뒤쪽에 있을 경우, 회전 없이 후진 주행합니다.
- 장애물 처리: `/drive_state` 토픽('run'/'stop'/'avoid')을 구독하여 회피 로직을 수행합니다.
- 배터리 관리: 배터리 상태를 구독하여 부족할 경우 새 임무를 거부합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64, String, Float32
from rcl_interfaces.msg import SetParametersResult
import math
import yaml
import os

from boltbot_state_machine.control_apps import PID


def normalize_angle(angle):
    """각도를 -π에서 +π 사이의 값으로 정규화합니다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def extract_pose_from_msg(msg: PoseStamped):
    """PoseStamped 메시지에서 x, y 위치와 yaw 각도를 추출합니다."""
    x = msg.pose.position.x
    y = msg.pose.position.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    yaw = 2 * math.atan2(qz, qw)
    return {'x': x, 'y': y, 'yaw': yaw}


class StateResult:
    CONTINUE = 0
    COMPLETE = 1


class ControllerState:
    def __init__(self, controller):
        self.controller = controller

    def update(self, current_pose):
        raise NotImplementedError("update()는 서브클래스에서 구현해야 합니다.")


class RotateToGoalState(ControllerState):
    def update(self, current_pose):
        twist = Twist()
        if self.controller.backward_drive:
            return twist, StateResult.COMPLETE
        dx = self.controller.goal_pose['x'] - current_pose['x']
        dy = self.controller.goal_pose['y'] - current_pose['y']
        desired = math.atan2(dy, dx)
        err = normalize_angle(desired - current_pose['yaw'])
        self.controller.angle_error_publisher.publish(Float64(data=err))
        self.controller.state_publisher.publish(String(data="RotateToGoal"))
        self.controller.get_logger().info(f"[Rotate] err={math.degrees(err):.1f}°")
        if abs(err) > self.controller.angle_tolerance:
            twist.angular.z = self.controller.angular_pid.update(err)
            return twist, StateResult.CONTINUE
        return twist, StateResult.COMPLETE


class MoveToGoalState(ControllerState):
    def update(self, current_pose):
        twist = Twist()
        dx = self.controller.goal_pose['x'] - current_pose['x']
        dy = self.controller.goal_pose['y'] - current_pose['y']
        if self.controller.use_euclidean_distance:
            dist = math.hypot(dx, dy)
        else:
            dist = dx * math.cos(current_pose['yaw']) + dy * math.sin(current_pose['yaw'])
        desired = math.atan2(dy, dx)
        raw_err = normalize_angle(desired - current_pose['yaw'])
        if abs(normalize_angle(raw_err + math.pi)) < abs(raw_err):
            dist = -dist
            err = normalize_angle(raw_err + math.pi)
        else:
            err = raw_err
        self.controller.distance_error_publisher.publish(Float64(data=dist))
        self.controller.angle_error_publisher.publish(Float64(data=err))
        self.controller.state_publisher.publish(String(data="MoveToGoal"))
        self.controller.get_logger().info(f"[Move] dist={dist:.2f}, err={math.degrees(err):.1f}°")
        if abs(dist) > self.controller.distance_tolerance:
            twist.linear.x = self.controller.linear_pid.update(dist)
            twist.angular.z = self.controller.angular_pid.update(err)
            return twist, StateResult.CONTINUE
        return twist, StateResult.COMPLETE


class RotateToFinalState(ControllerState):
    def update(self, current_pose):
        twist = Twist()
        err = normalize_angle(self.controller.goal_pose['yaw'] - current_pose['yaw'])
        self.controller.angle_error_publisher.publish(Float64(data=err))
        self.controller.state_publisher.publish(String(data="RotateToFinal"))
        self.controller.get_logger().info(f"[Final] err={math.degrees(err):.1f}°")
        if abs(err) > self.controller.angle_tolerance:
            twist.angular.z = self.controller.angular_pid.update(err)
            return twist, StateResult.CONTINUE
        return twist, StateResult.COMPLETE


class GoalReachedState(ControllerState):
    def update(self, current_pose):
        twist = Twist()
        self.controller.stop_robot()
        self.controller.state_publisher.publish(String(data="GoalReached"))
        if not getattr(self, 'logged', False):
            self.controller.get_logger().info("🎯 목표 도달!")
            self.logged = True
        return twist, StateResult.COMPLETE


class AvoidObstacleState(ControllerState):
    def update(self, current_pose):
        twist = Twist()
        bx = self.controller.box_center_x
        if bx is None:
            return twist, StateResult.CONTINUE
        mid = self.controller.image_width / 2.0
        twist.angular.z = -self.controller.avoid_angular_speed if bx < mid else self.controller.avoid_angular_speed
        self.controller.state_publisher.publish(String(data="AvoidObstacle"))
        return twist, StateResult.CONTINUE


class StateTransitionManager:
    def __init__(self, controller):
        self.controller = controller
    def get_next_state(self, state, res):
        if res == StateResult.COMPLETE:
            for pid in (self.controller.angular_pid, self.controller.linear_pid):
                pid.integrated_state = 0.0
                pid.pre_state = 0.0
            if isinstance(state, RotateToGoalState): return MoveToGoalState(self.controller)
            if isinstance(state, MoveToGoalState):   return RotateToFinalState(self.controller)
            if isinstance(state, RotateToFinalState): return GoalReachedState(self.controller)
            if isinstance(state, GoalReachedState):   return GoalReachedState(self.controller)
        return state


class RobotGoalController(Node):
    def __init__(self):
        super().__init__('robot_goal_controller')
        # 파라미터 선언
        self.declare_parameter('use_euclidean_distance', True)
        self.declare_parameter('avoid_angular_speed', 0.5)
        self.declare_parameter('image_width', 640)
        self.use_euclidean_distance = self.get_parameter('use_euclidean_distance').value
        self.avoid_angular_speed = self.get_parameter('avoid_angular_speed').value
        self.image_width = self.get_parameter('image_width').value
        self.add_on_set_parameters_callback(self.parameter_callback)
        # PID 컨트롤러 초기화
        self.pid_config_path = os.path.join(os.path.dirname(__file__), 'pid_config.yaml')
        self.angular_pid = PID()
        self.linear_pid = PID()
        self.update_parameters_from_yaml()
        # 장애물 및 상태 처리 변수
        self.drive_state = 'run'
        self.box_center_x = None
        self.battery_percentage = 100.0
        self.low_battery_threshold = 40.0
        
        # 구독 설정
        self.create_subscription(String, 'drive_state', self.drive_state_callback, 10)
        self.create_subscription(Float64, '/obstacle/box_center_x', self.box_center_callback, 10)
        self.create_subscription(PoseStamped, 'camera_pose', self.camera_pose_callback, 10)
        self.create_subscription(PoseStamped, 'target_pose', self.goal_pose_callback, 10)
        self.create_subscription(Float32, '/pinky_battery_present', self.battery_callback, 10)

        # 발행 설정
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.task_status_publisher = self.create_publisher(String, 'task_status', 10)

        # 상태 머신 초기화
        self.current_pose = None
        self.goal_pose = None
        self.backward_drive = False
        self.state_instance = None
        self.avoid_state = AvoidObstacleState(self)
        self.tm = StateTransitionManager(self)
        
        # 타이머
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.check_and_update_params)
        self.get_logger().info("🤖 RobotGoalController 시작")

    def parameter_callback(self, params):
        for p in params:
            if p.name == 'use_euclidean_distance':
                self.use_euclidean_distance = p.value
                self.get_logger().info(f"use_euclidean_distance → {p.value}")
        return SetParametersResult(successful=True)

    def drive_state_callback(self, msg: String):
        self.drive_state = msg.data
        self.get_logger().info(f"drive_state={self.drive_state}")

    def box_center_callback(self, msg: Float64):
        self.box_center_x = msg.data

    def battery_callback(self, msg: Float32):
        self.battery_percentage = msg.data
        if self.battery_percentage <= self.low_battery_threshold:
            self.get_logger().warn(f"Battery low: {self.battery_percentage:.1f}%")

    def update_parameters_from_yaml(self):
        try:
            with open(self.pid_config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.angle_tolerance = float(cfg['tolerances']['angle'])
            self.distance_tolerance = float(cfg['tolerances']['distance'])
            ap = cfg['angular']
            self.angular_pid.P = float(ap['P']); self.angular_pid.I = float(ap['I']); self.angular_pid.D = float(ap['D'])
            self.angular_pid.max_state = float(ap['max_state'])
            lp = cfg['linear']
            self.linear_pid.P = float(lp['P']); self.linear_pid.I = float(lp['I']); self.linear_pid.D = float(lp['D'])
            self.linear_pid.max_state = float(lp['max_state']); self.linear_pid.min_state = float(lp['min_state'])
            self.get_logger().info("✅ YAML 파라미터 로드 완료")
        except Exception as e:
            self.get_logger().error(f"❌ YAML 로드 실패: {e}")

    def check_and_update_params(self):
        try:
            m = os.path.getmtime(self.pid_config_path)
            if m > getattr(self, 'last_mtime', 0):
                self.last_mtime = m
                self.update_parameters_from_yaml()
        except:
            pass
            
    def goal_pose_callback(self, msg: PoseStamped):
        if self.battery_percentage <= self.low_battery_threshold:
            self.get_logger().error(f"Cannot start new goal. Battery is too low ({self.battery_percentage:.1f}%)!")
            self.task_status_publisher.publish(String(data="LOW_BATTERY"))
            return

        self.goal_pose = extract_pose_from_msg(msg)
        if self.current_pose:
            dx = self.goal_pose['x'] - self.current_pose['x']
            dy = self.goal_pose['y'] - self.current_pose['y']
            err = normalize_angle(math.atan2(dy, dx) - self.current_pose['yaw'])
            self.backward_drive = abs(err) > (math.pi / 2)
        else:
            self.backward_drive = False
        self.state_instance = MoveToGoalState(self) if self.backward_drive else RotateToGoalState(self)
        for pid in (self.angular_pid, self.linear_pid):
            pid.integrated_state = pid.pre_state = 0.0
        self.get_logger().info(f"새 목표 ({'후진' if self.backward_drive else '전진'})")

    def camera_pose_callback(self, msg: PoseStamped):
        self.current_pose = extract_pose_from_msg(msg)

    def control_loop(self):
        task_status_msg = String()
        
        # 1) 장애물 및 외부 정지 명령 우선 처리
        if self.drive_state == 'stop':
            self.stop_robot()
            task_status_msg.data = "IDLE"
            self.task_status_publisher.publish(task_status_msg)
            return
            
        if self.drive_state == 'avoid':
            twist = Twist()
            mid = self.image_width / 2.0
            if self.box_center_x is None: twist.linear.x = 0.0
            elif self.box_center_x < mid: twist.angular.z = -self.avoid_angular_speed
            else: twist.angular.z = self.avoid_angular_speed
            self.state_publisher.publish(String(data="AvoidObstacle"))
            self.cmd_vel_publisher.publish(twist)
            task_status_msg.data = "AVOIDING"
            self.task_status_publisher.publish(task_status_msg)
            return
            
        # 2) 목표 없는 대기 상태 처리
        if not (self.current_pose and self.goal_pose and self.state_instance):
            if not self.goal_pose:
                if self.battery_percentage <= self.low_battery_threshold:
                     task_status_msg.data = "LOW_BATTERY"
                else:
                     task_status_msg.data = "IDLE"
                self.task_status_publisher.publish(task_status_msg)
            return
            
        # 3) 일반 주행
        twist, res = self.state_instance.update(self.current_pose)
        self.state_instance = self.tm.get_next_state(self.state_instance, res)
        self.cmd_vel_publisher.publish(twist)

        # 4) 주행 중 상태 보고
        if isinstance(self.state_instance, GoalReachedState):
            task_status_msg.data = "SUCCEEDED"
            self.goal_pose = None
        else:
            task_status_msg.data = "MOVING"
        self.task_status_publisher.publish(task_status_msg)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        for pid in (self.angular_pid, self.linear_pid):
            pid.integrated_state = pid.pre_state = 0.0
        self.get_logger().info("⏹ 정지 & PID 리셋")


def main(args=None):
    rclpy.init(args=args)
    node = RobotGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("👋 종료 요청")
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()