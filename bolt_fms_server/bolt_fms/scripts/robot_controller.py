#!/usr/bin/env python3
"""
로봇 네비게이션 컨트롤러 (상태머신 기반)
- 목표 위치 구독
- PID 기반 네비게이션
- 상태머신으로 로봇 동작 제어
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32
from enum import Enum
import math
import time


class RobotState(Enum):
    """로봇 상태 정의"""
    IDLE = "idle"                    # 대기 상태
    NAVIGATING = "navigating"        # 네비게이션 중
    REACHED_GOAL = "reached_goal"    # 목표 도달
    ERROR = "error"                  # 에러 상태
    STOPPED = "stopped"              # 정지 상태


class PIDController:
    """간단한 PID 컨트롤러"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, error):
        """PID 제어 업데이트"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0.0:
            return 0.0
        
        # Proportional
        proportional = self.kp * error
        
        # Integral
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative
        derivative = self.kd * (error - self.prev_error) / dt
        
        # PID output
        output = proportional + integral + derivative
        
        # 출력 제한
        output = max(-self.max_output, min(self.max_output, output))
        
        # 이전 값 저장
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """PID 컨트롤러 초기화"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()


class RobotController(Node):
    """로봇 컨트롤러 노드"""
    
    def __init__(self, robot_id=1):
        super().__init__(f'robot_{robot_id}_controller')
        
        self.robot_id = robot_id
        self.state = RobotState.IDLE
        
        # 로봇 위치 및 목표
        self.current_pose = None
        self.target_pose = None
        self.battery_level = 1.0
        
        # 네비게이션 파라미터
        self.goal_tolerance = 0.1    # 목표 도달 허용 오차 (미터)
        self.max_linear_speed = 0.5  # 최대 선속도 (m/s)
        self.max_angular_speed = 1.0 # 최대 각속도 (rad/s)
        
        # PID 컨트롤러 설정
        self.distance_pid = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=self.max_linear_speed)
        self.angle_pid = PIDController(kp=2.0, ki=0.0, kd=0.2, max_output=self.max_angular_speed)
        
        # ROS2 구독자
        self.camera_pose_sub = self.create_subscription(
            PoseStamped,
            f'/robot{robot_id}/camera_pose',
            self.camera_pose_callback,
            10
        )
        
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            f'/robot{robot_id}/target_pose',
            self.target_pose_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            Float32,
            f'/robot{robot_id}/battery',
            self.battery_callback,
            10
        )
        
        # ROS2 발행자
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/robot{robot_id}/cmd_vel',
            10
        )
        
        # 상태머신 타이머 (20Hz)
        self.control_timer = self.create_timer(0.05, self.state_machine_update)
        
        # 로그 타이머 (1Hz)
        self.log_timer = self.create_timer(1.0, self.log_status)
        
        self.get_logger().info(f'🤖 Robot {robot_id} Controller initialized')
    
    def camera_pose_callback(self, msg):
        """카메라 포즈 콜백 (현재 위치)"""
        self.current_pose = msg.pose
    
    def target_pose_callback(self, msg):
        """목표 포즈 콜백"""
        self.target_pose = msg.pose
        
        # 새로운 목표가 설정되면 네비게이션 상태로 전환
        if self.state in [RobotState.IDLE, RobotState.REACHED_GOAL]:
            self.state = RobotState.NAVIGATING
            self.distance_pid.reset()
            self.angle_pid.reset()
            
            target_x = msg.pose.position.x
            target_y = msg.pose.position.y
            self.get_logger().info(f'🎯 New target received: ({target_x:.2f}, {target_y:.2f})')
    
    def battery_callback(self, msg):
        """배터리 레벨 콜백"""
        self.battery_level = msg.data
        
        # 배터리가 너무 낮으면 에러 상태로 전환
        if self.battery_level < 0.2:  # 20% 미만
            if self.state != RobotState.ERROR:
                self.state = RobotState.ERROR
                self.get_logger().warn(f'🔋 Low battery: {self.battery_level*100:.1f}%')
    
    def state_machine_update(self):
        """상태머신 메인 루프"""
        if self.state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.state == RobotState.NAVIGATING:
            self.handle_navigating_state()
        elif self.state == RobotState.REACHED_GOAL:
            self.handle_reached_goal_state()
        elif self.state == RobotState.ERROR:
            self.handle_error_state()
        elif self.state == RobotState.STOPPED:
            self.handle_stopped_state()
    
    def handle_idle_state(self):
        """대기 상태 처리"""
        # 정지 명령 발행
        self.publish_stop_command()
        
        # 목표가 설정되면 네비게이션 상태로 전환
        if self.target_pose is not None:
            self.state = RobotState.NAVIGATING
            self.get_logger().info('📍 Starting navigation')
    
    def handle_navigating_state(self):
        """네비게이션 상태 처리"""
        if self.current_pose is None or self.target_pose is None:
            return
        
        # 목표까지의 거리 및 각도 계산
        distance, angle_error = self.calculate_navigation_errors()
        
        # 목표 도달 확인
        if distance <= self.goal_tolerance:
            self.state = RobotState.REACHED_GOAL
            self.get_logger().info(f'✅ Goal reached! Distance: {distance:.3f}m')
            return
        
        # PID 제어로 속도 계산
        linear_velocity = self.distance_pid.update(distance)
        angular_velocity = self.angle_pid.update(angle_error)
        
        # 속도 제한 적용
        linear_velocity = max(-self.max_linear_speed, min(self.max_linear_speed, linear_velocity))
        angular_velocity = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        
        # 회전이 큰 경우 선속도 감소
        if abs(angle_error) > math.pi/4:  # 45도 이상
            linear_velocity *= 0.5
        
        # 속도 명령 발행
        self.publish_velocity_command(linear_velocity, angular_velocity)
    
    def handle_reached_goal_state(self):
        """목표 도달 상태 처리"""
        # 정지 명령 발행
        self.publish_stop_command()
        
        # 잠시 대기 후 IDLE 상태로 전환
        self.state = RobotState.IDLE
        self.target_pose = None  # 목표 초기화
    
    def handle_error_state(self):
        """에러 상태 처리"""
        # 정지 명령 발행
        self.publish_stop_command()
        
        # 배터리가 회복되면 IDLE 상태로 전환
        if self.battery_level >= 0.3:  # 30% 이상
            self.state = RobotState.IDLE
            self.get_logger().info('🔋 Battery recovered, returning to IDLE')
    
    def handle_stopped_state(self):
        """정지 상태 처리"""
        self.publish_stop_command()
    
    def calculate_navigation_errors(self):
        """네비게이션 에러 계산 (거리, 각도)"""
        # 현재 위치
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 현재 방향 (yaw)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # 목표 위치
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y
        
        # 거리 계산
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 목표 각도 계산
        target_yaw = math.atan2(dy, dx)
        
        # 각도 에러 계산 (-π ~ π 범위로 정규화)
        angle_error = target_yaw - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        return distance, angle_error
    
    def quaternion_to_yaw(self, quaternion):
        """쿼터니언을 yaw 각도로 변환"""
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def publish_velocity_command(self, linear_vel, angular_vel):
        """속도 명령 발행"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(twist_msg)
    
    def publish_stop_command(self):
        """정지 명령 발행"""
        self.publish_velocity_command(0.0, 0.0)
    
    def log_status(self):
        """상태 로그 출력"""
        if self.current_pose is None:
            return
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        status_msg = f'🤖 Robot{self.robot_id} | State: {self.state.value} | '
        status_msg += f'Pos: ({current_x:.2f}, {current_y:.2f}) | '
        status_msg += f'Yaw: {math.degrees(current_yaw):.1f}° | '
        status_msg += f'Battery: {self.battery_level*100:.1f}%'
        
        if self.target_pose is not None:
            target_x = self.target_pose.position.x
            target_y = self.target_pose.position.y
            distance, _ = self.calculate_navigation_errors()
            status_msg += f' | Target: ({target_x:.2f}, {target_y:.2f}) | Dist: {distance:.2f}m'
        
        self.get_logger().info(status_msg)


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    # 로봇 ID를 인자로 받기 (기본값: 1)
    import sys
    robot_id = 3
    if len(sys.argv) > 1:
        try:
            robot_id = int(sys.argv[1])
        except ValueError:
            print("Invalid robot ID. Using default: 1")
    
    print(f"🚀 Starting Robot {robot_id} Controller...")
    
    try:
        controller = RobotController(robot_id)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print(f"\n⏹️ Robot {robot_id} Controller stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()