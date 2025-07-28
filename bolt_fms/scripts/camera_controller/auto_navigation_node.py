#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np

class AutoNavigationNode(Node):
    def __init__(self):
        super().__init__('auto_navigation_node')
        
        # QoS 프로필 설정
        qos = QoSProfile(depth=10)
        
        # Publisher - cmd_vel로 이동 명령 발행
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        
        # Subscribers
        # camera_pose, 메시지 타입 PoseStamped
        self.camera_pose_sub = self.create_subscription(
            PoseStamped,
            'camera_pose',
            self.camera_pose_callback,
            qos
        )
        
        # target_pose, 메시지 타입 PoseStamped
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.target_pose_callback,
            qos
        )
        
        # 제어 파라미터
        self.MAX_LIN_VEL = 0.5
        self.MAX_ANG_VEL = 0.5
        self.LIN_VEL_STEP_SIZE = 0.1
        self.ANG_VEL_STEP_SIZE = 0.05
        
        # 제어 변수
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        
        # 위치 정보
        self.current_pose = None
        self.target_pose = None
        self.is_moving = False
        
        # 임계값 설정 (미터 단위)
        self.position_threshold = 0.1  # 10cm 이내 도착으로 간주
        self.angle_threshold = 0.1     # 약 5.7도 이내 각도 오차 허용
        
        # 제어 게인
        self.kp_linear = 0.5   # 선속도 비례 게인
        self.kp_angular = 1.0  # 각속도 비례 게인

        self.state = 'rotate_to_target'  # or 'go_straight', 'final_orientation'

        
        # 타이머 - 20Hz로 제어 루프 실행
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("자동 네비게이션 노드가 시작되었습니다.")
        #  토픽 기달 로그 
        self.get_logger().info("camera_pose와 target_pose 토픽을 기다리는 중...")

    def camera_pose_callback(self, msg):
        """현재 위치 콜백"""
        self.current_pose = self.extract_pose_from_msg(msg)
        self.get_logger().info(f"현재 위치!!!!!: x={self.current_pose['x']:.3f}, y={self.current_pose['y']:.3f}, yaw={self.current_pose['yaw']:.3f}")
        
    def target_pose_callback(self, msg):
        """목표 위치 콜백"""
        self.target_pose = self.extract_pose_from_msg(msg)
        self.is_moving = True
        self.get_logger().info(f"새로운 목표점 수신: x={self.target_pose['x']:.3f}, y={self.target_pose['y']:.3f}, yaw={self.target_pose['yaw']:.3f}")

    def extract_pose_from_msg(self, msg):
        """PoseStamped 메시지에서 x, y, yaw 추출 (PoseWithCovarianceStamped에서 변경)"""
        # PoseStamped 메시지 구조에 맞게 .pose.pose를 .pose로 수정
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # 쿼터니언을 오일러 각으로 변환
        # PoseStamped 메시지 구조에 맞게 .pose.pose를 .pose로 수정
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2 * math.atan2(qz, qw)
        
        return {'x': x, 'y': y, 'yaw': yaw}

    def calculate_distance(self, pose1, pose2):
        """두 점 사이의 거리 계산"""
        dx = pose2['x'] - pose1['x']
        dy = pose2['y'] - pose1['y']
        return math.sqrt(dx**2 + dy**2)

    def calculate_angle_to_target(self, current_pose, target_pose):
        """현재 위치에서 목표점으로의 각도 계산"""
        dx = target_pose['x'] - current_pose['x']
        dy = target_pose['y'] - current_pose['y']
        target_angle = math.atan2(dy, dx)
        return target_angle

    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def make_simple_profile(self, output, input_val, slop):
        """부드러운 속도 변화를 위한 프로파일 생성"""
        if input_val > output:
            output = min(input_val, output + slop)
        elif input_val < output:
            output = max(input_val, output - slop)
        else:
            output = input_val
        return output

    def constrain(self, input_vel, low_bound, high_bound):
        """속도 제한"""
        return max(low_bound, min(high_bound, input_vel))

    def control_loop(self):
        if not self.is_moving or self.current_pose is None or self.target_pose is None:
            return

        current = self.current_pose
        target = self.target_pose
        distance = self.calculate_distance(current, target)
        angle_to_target = self.calculate_angle_to_target(current, target)
        angle_error = self.normalize_angle(angle_to_target - current['yaw'])
        final_yaw_error = self.normalize_angle(target['yaw'] - current['yaw'])

        twist = Twist()

        if self.state == 'rotate_to_target':
            if abs(angle_error) > self.angle_threshold:
                # 제자리 회전
                twist.angular.z = self.kp_angular * angle_error
                twist.angular.z = self.constrain(twist.angular.z, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)
            else:
                self.state = 'go_straight'
                self.get_logger().info("✅ 목표 방향 정렬 완료 → 직진 시작")

        elif self.state == 'go_straight':
            if distance > self.position_threshold:
                # 직진
                twist.linear.x = min(self.kp_linear * distance, self.MAX_LIN_VEL)
                twist.angular.z = self.kp_angular * angle_error * 0.3  # 경로 이탈 미세 조정
            else:
                self.state = 'final_orientation'
                self.get_logger().info("✅ 목표 위치 도착 → 최종 방향 정렬 시작")

        elif self.state == 'final_orientation':
            if abs(final_yaw_error) > self.angle_threshold:
                twist.angular.z = self.kp_angular * final_yaw_error
                twist.angular.z = self.constrain(twist.angular.z, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)
            else:
                self.get_logger().info("🎯 목표 방향까지 정렬 완료!")
                self.stop_robot()
                self.is_moving = False
                self.state = 'rotate_to_target'  # 초기화

        self.cmd_vel_pub.publish(twist)

        
        # 주기적으로 상태 출력
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 20 == 0:  # 1초마다 로그 출력
            self.get_logger().info(
                f"거리: {distance:.3f}m, 각도오차: {math.degrees(angle_error):.1f}°, "
                f"선속도: {self.current_linear_velocity:.2f}, 각속도: {self.current_angular_velocity:.2f}")

    def stop_robot(self):
        """로봇 정지"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
        # 속도 변수 초기화
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        auto_nav_node = AutoNavigationNode()
        rclpy.spin(auto_nav_node)
    except KeyboardInterrupt:
        print("사용자에 의해 종료되었습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        if 'auto_nav_node' in locals():
            auto_nav_node.stop_robot()
            auto_nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()