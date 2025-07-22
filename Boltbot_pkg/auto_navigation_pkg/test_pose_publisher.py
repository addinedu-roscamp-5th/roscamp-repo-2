#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# PoseStamped 메시지 타입을 사용하기 위해 import 변경
from geometry_msgs.msg import PoseStamped
import math
import time

class TestPosePublisher(Node):
    def __init__(self):
        super().__init__('test_pose_publisher')
        
        qos = QoSProfile(depth=10)
        
        # Publishers
        # camera_pose 토픽의 메시지 타입을 PoseStamped로 변경
        self.camera_pose_pub = self.create_publisher(
            PoseStamped, 'camera_pose', qos)
        # target_pose를 goal_pose로 변경하고 메시지 타입을 PoseStamped로 변경
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, 'goal_pose', qos)
        
        # 테스트용 타이머
        self.timer = self.create_timer(0.1, self.publish_camera_pose)  # 10Hz
        
        # 현재 위치 (시뮬레이션)
        self.current_x = 0.0
        self.current_y = 0.9
        self.current_yaw = 0.0
        
        self.get_logger().info("테스트 Pose Publisher 시작")
        
        # 5초 후 목표점 발행
        # publish_target_pose를 publish_goal_pose로 변경
        self.create_timer(5.0, self.publish_goal_pose)

    def create_pose_msg(self, x, y, yaw):
        """PoseStamped 메시지 생성 (PoseWithCovarianceStamped에서 변경)"""
        now = self.get_clock().now()
        msg = PoseStamped() # PoseStamped 메시지 생성
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        # PoseStamped 메시지 구조에 맞게 .pose.pose를 .pose로 수정
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        # PoseStamped는 covariance 필드가 없으므로 관련 코드 삭제
        
        return msg

    def publish_camera_pose(self):
        """현재 위치 발행 (시뮬레이션)"""
        msg = self.create_pose_msg(self.current_x, self.current_y, self.current_yaw)
        self.camera_pose_pub.publish(msg)

    def publish_goal_pose(self): # 함수 이름 변경: publish_target_pose -> publish_goal_pose
        """목표 위치 발행"""
        # 사용자가 제공한 스크립트의 값으로 목표 위치 설정
        target_x = 1.0
        target_y = 0.13
        # z=0.707, w=0.707는 yaw가 pi/2 (90도)인 경우에 해당
        target_yaw = math.pi / 2.0
        
        msg = self.create_pose_msg(target_x, target_y, target_yaw)
        # goal_pose_pub으로 발행
        self.goal_pose_pub.publish(msg)
        
        self.get_logger().info(f"목표점 발행: x={target_x}, y={target_y}, yaw={target_yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
