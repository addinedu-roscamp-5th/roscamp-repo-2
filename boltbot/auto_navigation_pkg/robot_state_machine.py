#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
이 스크립트는 로봇을 특정 목표 지점(x, y, yaw)으로 이동시키기 위한 상태 머신 기반의 제어기입니다.
로봇의 현재 위치는 `/camera_pose` 토픽에서, 목표 위치는 `/target_pose` 토픽에서 받아옵니다.
계산된 속도 명령은 `/cmd_vel` 토픽으로 발행됩니다.

주요 기능:
- 상태 기반 제어: 로봇의 행동을 '목표 방향으로 회전', '목표 지점으로 이동', '최종 각도로 회전', '목표 도달'의 4가지 상태로 나누어 관리합니다.
- PID 제어: 선속도와 각속도를 제어하기 위해 PID(Proportional-Integral-Derivative) 제어기를 사용합니다.
- YAML 기반 동적 파라미터 설정: 'pid_config.yaml' 파일에서 PID 게인 값, 허용 오차 등을 실시간으로 읽어와 반영합니다.
- 상태 모니터링: 현재 상태, 거리 오차, 각도 오차를 토픽으로 발행하여 외부에서 모니터링할 수 있습니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import SetParametersResult
import math
import time
import yaml
import os

# control_apps 라이브러리에서 PID 컨트롤러 클래스를 가져옵니다.
from controller_tutorials.control_apps import PID

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

# 상태 결과 정의
class StateResult:
    CONTINUE = 0
    COMPLETE = 1

# 추상 상태 클래스
class ControllerState:
    def __init__(self, controller):
        self.controller = controller

    def update(self, current_pose):
        raise NotImplementedError("update()는 서브클래스에서 구현해야 합니다.")

# 상태 1: RotateToGoal
class RotateToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        desired_heading = math.atan2(
            self.controller.goal_pose['y'] - current_pose['y'],
            self.controller.goal_pose['x'] - current_pose['x'])
        error_angle = normalize_angle(desired_heading - current_pose['yaw'])
        
        self.controller.angle_error_publisher.publish(Float64(data=error_angle))
        self.controller.state_publisher.publish(String(data="RotateToGoal"))
        self.controller.get_logger().info(f"[RotateToGoal] 각도 오차: {math.degrees(error_angle):.2f}°")

        if abs(error_angle) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            return twist_msg, StateResult.CONTINUE
        else:
            self.controller.get_logger().info("목표 방향 정렬 완료.")
            return twist_msg, StateResult.COMPLETE

# 상태 2: MoveToGoal
class MoveToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        dx = self.controller.goal_pose['x'] - current_pose['x']
        dy = self.controller.goal_pose['y'] - current_pose['y']
        
        # 파라미터 값에 따라 거리 계산 방식 선택
        if self.controller.use_euclidean_distance:
            # 방식 1: 유클리드 거리 (로봇 방향과 무관한 절대 거리)
            distance_error = math.sqrt(dx**2 + dy**2)
        else:
            # 방식 2: 투영 거리 (로봇의 현재 진행 방향 기준 거리)
            distance_error = dx * math.cos(current_pose['yaw']) + dy * math.sin(current_pose['yaw'])
        
        self.controller.distance_error_publisher.publish(Float64(data=distance_error))
        self.controller.state_publisher.publish(String(data="MoveToGoal"))
        self.controller.get_logger().info(f"[MoveToGoal] 거리 오차: {distance_error:.2f}m (유클리드: {self.controller.use_euclidean_distance})")

        if abs(distance_error) > self.controller.distance_tolerance:
            linear_correction = self.controller.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            
            desired_heading = math.atan2(dy, dx)
            angle_error = normalize_angle(desired_heading - current_pose['yaw'])
            angular_correction = self.controller.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
            return twist_msg, StateResult.CONTINUE
        else:
            self.controller.get_logger().info("목표 위치 도달.")
            return twist_msg, StateResult.COMPLETE

# 상태 3: RotateToFinal
class RotateToFinalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        final_error = normalize_angle(self.controller.goal_pose['yaw'] - current_pose['yaw'])
        
        self.controller.angle_error_publisher.publish(Float64(data=final_error))
        self.controller.state_publisher.publish(String(data="RotateToFinal"))
        self.controller.get_logger().info(f"[RotateToFinal] 최종 각도 오차: {math.degrees(final_error):.2f}°")

        if abs(final_error) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction
            return twist_msg, StateResult.CONTINUE
        else:
            self.controller.get_logger().info("최종 방향 정렬 완료.")
            return twist_msg, StateResult.COMPLETE

# 상태 4: GoalReached
class GoalReachedState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        self.controller.stop_robot()
        self.controller.state_publisher.publish(String(data="GoalReached"))
        if not getattr(self, 'goal_reached_logged', False):
            self.controller.get_logger().info("최종 목표 도달!")
            self.goal_reached_logged = True
        return twist_msg, StateResult.COMPLETE

# 상태 전환 관리자
class StateTransitionManager:
    def __init__(self, controller):
        self.controller = controller

    def get_next_state(self, current_state, state_result):
        if state_result == StateResult.COMPLETE:
            if isinstance(current_state, RotateToGoalState):
                self.controller.angular_pid.integrated_state = 0.0
                self.controller.angular_pid.pre_state = 0.0
                return MoveToGoalState(self.controller)
            elif isinstance(current_state, MoveToGoalState):
                self.controller.linear_pid.integrated_state = 0.0
                self.controller.linear_pid.pre_state = 0.0
                self.controller.angular_pid.integrated_state = 0.0
                self.controller.angular_pid.pre_state = 0.0
                return RotateToFinalState(self.controller)
            elif isinstance(current_state, RotateToFinalState):
                self.controller.angular_pid.integrated_state = 0.0
                self.controller.angular_pid.pre_state = 0.0
                return GoalReachedState(self.controller)
            elif isinstance(current_state, GoalReachedState):
                return GoalReachedState(self.controller)
        return current_state

# 메인 제어기 노드
class RobotGoalController(Node):
    def __init__(self):
        super().__init__('robot_goal_controller')
        
        # ROS 파라미터 선언
        self.declare_parameter('use_euclidean_distance', True)
        self.use_euclidean_distance = self.get_parameter('use_euclidean_distance').value
        self.add_on_set_parameters_callback(self.parameter_callback)

        # PID 설정 파일 경로
        self.pid_config_path = os.path.join(os.path.dirname(__file__), 'pid_config.yaml')
        self.last_mtime = 0

        # PID 컨트롤러 생성 및 초기화
        self.angular_pid = PID()
        self.linear_pid = PID()
        self.update_parameters_from_yaml()

        self.current_pose = None
        self.goal_pose = None
        self.state_instance = None
        self.state_transition_manager = StateTransitionManager(self)
        
        self.pose_subscriber = self.create_subscription(PoseStamped, 'camera_pose', self.camera_pose_callback, 10)
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, 'target_pose', self.goal_pose_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.param_check_timer = self.create_timer(1.0, self.check_and_update_params) # 1초마다 파일 변경 확인
        self.get_logger().info("로봇 목표 제어기(상태 머신)가 시작되었습니다.")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'use_euclidean_distance':
                self.use_euclidean_distance = param.value
                self.get_logger().info(f"'use_euclidean_distance' 파라미터가 {self.use_euclidean_distance}(으)로 변경되었습니다.")
        return SetParametersResult(successful=True)

    def update_parameters_from_yaml(self):
        """YAML 파일에서 모든 파라미터를 읽어와 업데이트합니다."""
        try:
            with open(self.pid_config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.angle_tolerance = float(config['tolerances']['angle'])
            self.distance_tolerance = float(config['tolerances']['distance'])

            self.angular_pid.P = float(config['angular']['P'])
            self.angular_pid.I = float(config['angular']['I'])
            self.angular_pid.D = float(config['angular']['D'])
            self.angular_pid.max_state = float(config['angular']['max_state'])
            self.angular_pid.min_state = float(config['angular']['min_state'])
            
            self.linear_pid.P = float(config['linear']['P'])
            self.linear_pid.I = float(config['linear']['I'])
            self.linear_pid.D = float(config['linear']['D'])
            self.linear_pid.max_state = float(config['linear']['max_state'])
            self.linear_pid.min_state = float(config['linear']['min_state'])
            
            self.get_logger().info("모든 파라미터가 YAML 파일로부터 업데이트되었습니다.")
            return True
        except (IOError, yaml.YAMLError, KeyError) as e:
            self.get_logger().error(f"설정 파일('.yaml')을 읽는 중 오류 발생: {e}")
            return False

    def check_and_update_params(self):
        """설정 파일의 변경을 감지하고 파라미터를 업데이트합니다."""
        try:
            mtime = os.path.getmtime(self.pid_config_path)
            if mtime > self.last_mtime:
                self.last_mtime = mtime
                self.get_logger().info("설정 파일 변경 감지. 파라미터를 다시 로드합니다.")
                self.update_parameters_from_yaml()
        except OSError as e:
            self.get_logger().warn(f"설정 파일('.yaml')에 접근할 수 없습니다: {e}")

    def goal_pose_callback(self, msg):
        self.goal_pose = extract_pose_from_msg(msg)
        self.state_instance = RotateToGoalState(self)
        self.linear_pid.integrated_state = 0.0
        self.linear_pid.pre_state = 0.0
        self.angular_pid.integrated_state = 0.0
        self.angular_pid.pre_state = 0.0
        self.get_logger().info(
            f"새로운 목표 수신: x={self.goal_pose['x']:.2f}, y={self.goal_pose['y']:.2f}, yaw={math.degrees(self.goal_pose['yaw']):.2f}°"
        )
    
    def camera_pose_callback(self, msg):
        self.current_pose = extract_pose_from_msg(msg)

    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None or self.state_instance is None:
            return
        
        twist_msg, status = self.state_instance.update(self.current_pose)
        self.state_instance = self.state_transition_manager.get_next_state(self.state_instance, status)
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.linear_pid.integrated_state = 0.0
        self.linear_pid.pre_state = 0.0
        self.angular_pid.integrated_state = 0.0
        self.angular_pid.pre_state = 0.0
        self.get_logger().info("로봇 정지 및 PID 상태 초기화.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드가 중단되었습니다.")
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

