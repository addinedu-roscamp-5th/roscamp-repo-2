#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
이 스크립트는 turtlesim 대신 실제 로봇을 제어하기 위해 move_turtle_state_machine.py를 수정한 버전입니다.
- 목적: 목표 지점(target_pose)을 향해 로봇을 이동시키는 상태 기반 제어기입니다.
- 상태: RotateToGoal -> MoveToGoal -> RotateToFinal -> GoalReached 순서로 진행됩니다.
- 수정 사항:
  - turtlesim.msg.Pose 대신 geometry_msgs.msg.PoseStamped 메시지를 사용합니다.
  - /turtle1/pose, /goal_pose 토픽 대신 /camera_pose, /target_pose 토픽을 구독합니다.
  - /turtle1/cmd_vel 대신 /cmd_vel 토픽으로 제어 명령을 발행합니다.
  - auto_navigation_node.py의 좌표 추출 및 오차 계산 방식을 일부 적용하여 실제 환경에 맞게 수정했습니다.
  - **auto_navigation_node.py처럼 주기적인 제어 루프를 추가하여 제어 명령이 지속적으로 발행되도록 수정했습니다.**
- 의존성: rclpy, geometry_msgs, std_msgs, controller_tutorials.control_apps
- 참조 위치:
  - 상태 머신 구조: controller_tutorials/move_turtle_state_machine.py
  - PoseStamped 처리 및 PID 로직: auto_navigation_pkg/auto_navigation_node.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import SetParametersResult
import math
import time

# control_apps.py의 PID 클래스를 그대로 사용합니다.
from controller_tutorials.control_apps import PID

def normalize_angle(angle):
    """ 각도를 -π ~ π 범위로 정규화합니다. """
    return math.atan2(math.sin(angle), math.cos(angle))

def extract_pose_from_msg(msg: PoseStamped):
    """ PoseStamped 메시지에서 x, y, yaw를 추출하여 딕셔너리로 반환합니다. """
    x = msg.pose.position.x
    y = msg.pose.position.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    # 쿼터니언에서 yaw 각도 계산
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

# 상태 1: 목표 지점을 향해 회전 (RotateToGoal)
class RotateToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        # 목표점 방향 계산
        desired_heading = math.atan2(
            self.controller.goal_pose['y'] - current_pose['y'],
            self.controller.goal_pose['x'] - current_pose['x'])
        # 각도 오차 계산
        error_angle = normalize_angle(desired_heading - current_pose['yaw'])
        
        error_msg = Float64()
        error_msg.data = error_angle
        self.controller.angle_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="RotateToGoal"))
        self.controller.get_logger().info(f"[RotateToGoal] 목표 방향 각도 오차: {math.degrees(error_angle):.2f}°")

        # 오차가 허용치보다 크면 회전
        if abs(error_angle) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else: # 회전 완료
            self.controller.get_logger().info("목표 방향 정렬 완료. 다음 상태로 전환합니다.")
            return twist_msg, StateResult.COMPLETE

# 상태 2: 목표 지점으로 이동 (MoveToGoal)
class MoveToGoalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        dx = self.controller.goal_pose['x'] - current_pose['x']
        dy = self.controller.goal_pose['y'] - current_pose['y']
        
        # auto_navigation_node.py와 유사한 거리 오차 계산 방식 적용
        distance_error = dx * math.cos(current_pose['yaw']) + dy * math.sin(current_pose['yaw'])
        
        error_msg = Float64()
        error_msg.data = distance_error
        self.controller.distance_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="MoveToGoal"))
        self.controller.get_logger().info(f"[MoveToGoal] 목표까지 거리 오차: {distance_error:.2f}m")

        # 오차가 허용치보다 크면 이동
        if abs(distance_error) > self.controller.distance_tolerance:
            # 선속도 제어
            linear_correction = self.controller.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            
            # 이동 중 방향 보정 (각속도 제어)
            desired_heading = math.atan2(dy, dx)
            angle_error = normalize_angle(desired_heading - current_pose['yaw'])
            angular_correction = self.controller.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
            return twist_msg, StateResult.CONTINUE
        else: # 이동 완료
            self.controller.get_logger().info("목표 위치 도달. 다음 상태로 전환합니다.")
            return twist_msg, StateResult.COMPLETE

# 상태 3: 최종 목표 각도로 회전 (RotateToFinal)
class RotateToFinalState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        # 최종 각도 오차 계산
        final_error = normalize_angle(self.controller.goal_pose['yaw'] - current_pose['yaw'])
        
        error_msg = Float64()
        error_msg.data = final_error
        self.controller.angle_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="RotateToFinal"))
        self.controller.get_logger().info(f"[RotateToFinal] 최종 각도 오차: {math.degrees(final_error):.2f}°")

        # 오차가 허용치보다 크면 회전
        if abs(final_error) > self.controller.angle_tolerance:
            angular_correction = self.controller.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else: # 회전 완료
            self.controller.get_logger().info("최종 방향 정렬 완료. 다음 상태로 전환합니다.")
            return twist_msg, StateResult.COMPLETE

# 상태 4: 목표 도달 (GoalReached)
class GoalReachedState(ControllerState):
    def update(self, current_pose):
        twist_msg = Twist()
        # 목표 도달 시 로봇 정지 및 PID 상태 초기화
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
                # 상태 전환 시 PID 상태 초기화
                self.controller.angular_pid.integrated_state = 0.0
                self.controller.angular_pid.pre_state = 0.0
                return MoveToGoalState(self.controller)
            elif isinstance(current_state, MoveToGoalState):
                # 상태 전환 시 PID 상태 초기화
                self.controller.linear_pid.integrated_state = 0.0
                self.controller.linear_pid.pre_state = 0.0
                self.controller.angular_pid.integrated_state = 0.0
                self.controller.angular_pid.pre_state = 0.0
                return RotateToFinalState(self.controller)
            elif isinstance(current_state, RotateToFinalState):
                # 상태 전환 시 PID 상태 초기화
                self.controller.angular_pid.integrated_state = 0.0
                self.controller.angular_pid.pre_state = 0.0
                return GoalReachedState(self.controller)
            elif isinstance(current_state, GoalReachedState):
                return GoalReachedState(self.controller) # 최종 상태 유지
        return current_state # CONTINUE일 경우 현재 상태 유지

# 메인 제어기 노드
class RobotGoalController(Node):
    def __init__(self):
        super().__init__('robot_goal_controller')
        
        # 파라미터 선언 (auto_navigation_node.py 참조)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angular_P', 5.0)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        self.declare_parameter('angular_max_state', 1.0)
        self.declare_parameter('angular_min_state', -1.0) # 음수 값 허용
        self.declare_parameter('linear_P', 1.0)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 0.5)
        self.declare_parameter('linear_min_state', -0.5) # 음수 값 허용

        # PID 컨트롤러 생성 및 초기화
        self.angular_pid = PID()
        self.linear_pid = PID()
        self.update_pid_parameters() # 파라미터 값으로 PID 초기화

        self.current_pose = None
        self.goal_pose = None
        self.state_instance = None
        self.state_transition_manager = StateTransitionManager(self)
        
        # 구독자: /camera_pose와 /target_pose를 구독
        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'camera_pose', self.camera_pose_callback, 10) # 이름 변경
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped, 'target_pose', self.goal_pose_callback, 10)
        
        # 발행자: /cmd_vel로 제어 명령 발행
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        
        # 주기적인 제어 루프 타이머 (auto_navigation_node.py 참고)
        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info("로봇 목표 제어기(상태 머신)가 시작되었습니다.")

    def update_pid_parameters(self):
        """ 파라미터 값으로 PID 컨트롤러를 업데이트하는 함수 """
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        self.angular_pid.P = self.get_parameter('angular_P').value
        self.angular_pid.I = self.get_parameter('angular_I').value
        self.angular_pid.D = self.get_parameter('angular_D').value
        self.angular_pid.max_state = self.get_parameter('angular_max_state').value
        self.angular_pid.min_state = self.get_parameter('angular_min_state').value

        self.linear_pid.P = self.get_parameter('linear_P').value
        self.linear_pid.I = self.get_parameter('linear_I').value
        self.linear_pid.D = self.get_parameter('linear_D').value
        self.linear_pid.max_state = self.get_parameter('linear_max_state').value
        self.linear_pid.min_state = self.get_parameter('linear_min_state').value
        self.get_logger().info("PID 파라미터가 업데이트되었습니다.")

    def parameter_callback(self, params):
        """ 동적으로 PID 파라미터를 업데이트하는 콜백 함수 """
        # 모든 파라미터가 업데이트 될 때마다 PID 값을 다시 로드
        self.update_pid_parameters()
        return SetParametersResult(successful=True)
    
    def goal_pose_callback(self, msg):
        """ 새로운 목표 지점 콜백 함수 """
        self.goal_pose = extract_pose_from_msg(msg)
        # 새로운 목표가 설정되면 상태 머신을 초기 상태로 재설정
        self.state_instance = RotateToGoalState(self)
        # PID 상태 초기화
        self.linear_pid.integrated_state = 0.0
        self.linear_pid.pre_state = 0.0
        self.angular_pid.integrated_state = 0.0
        self.angular_pid.pre_state = 0.0

        self.get_logger().info(
            f"새로운 목표 수신: x={self.goal_pose['x']:.2f}, y={self.goal_pose['y']:.2f}, yaw={math.degrees(self.goal_pose['yaw']):.2f}°"
        )
    
    def camera_pose_callback(self, msg):
        """ 현재 위치 콜백 함수 (이름 변경) """
        self.current_pose = extract_pose_from_msg(msg)

    def control_loop(self):
        """ 주기적인 제어 루프 (auto_navigation_node.py 참고) """
        if self.current_pose is None or self.goal_pose is None or self.state_instance is None:
            # 필요한 정보가 없으면 제어하지 않음
            return
        
        # 상태 머신 업데이트 및 제어 명령 발행
        twist_msg, status = self.state_instance.update(self.current_pose)
        self.state_instance = self.state_transition_manager.get_next_state(self.state_instance, status)
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """ 로봇을 정지시키고 PID 상태를 초기화하는 함수 """
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        # PID 내부 상태 초기화
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
        # 종료 시 로봇 정지
        if rclpy.ok(): # 노드가 아직 유효한 경우에만 정지 명령 발행
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()