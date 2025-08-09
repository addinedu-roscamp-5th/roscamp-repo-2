#!/usr/bin/env python3
"""
로봇 상태 구독자 (RobotStatusSubscriber)
- 여러 로봇의 다양한 토픽을 구독하여 상태 모니터링
- camera_pose, target_pose, cmd_vel, battery, task, state 등을 통합 관리
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from typing import Dict, Optional, List
import time
import math
import json


@dataclass
class RobotStatus:
    """로봇 상태 정보를 담는 데이터 클래스"""
    robot_id: int
    
    # 위치 정보
    camera_pose: Optional[PoseStamped] = None
    target_pose: Optional[PoseStamped] = None
    
    # 제어 정보
    cmd_vel: Optional[Twist] = None
    
    # 시스템 정보
    battery: Optional[float] = None
    task: Optional[str] = None
    state: Optional[str] = None
    
    # 로봇팔 정보 (해당하는 경우)
    joint_states: Optional[JointState] = None
    
    # 타임스탬프
    last_camera_pose_time: float = 0.0
    last_target_pose_time: float = 0.0
    last_cmd_vel_time: float = 0.0
    last_battery_time: float = 0.0
    last_task_time: float = 0.0
    last_state_time: float = 0.0
    last_joint_states_time: float = 0.0
    
    def is_camera_pose_recent(self, timeout=2.0) -> bool:
        """카메라 포즈가 최근 데이터인지 확인"""
        return (time.time() - self.last_camera_pose_time) < timeout
    
    def is_target_pose_recent(self, timeout=5.0) -> bool:
        """타겟 포즈가 최근 데이터인지 확인"""
        return (time.time() - self.last_target_pose_time) < timeout
    
    def is_cmd_vel_recent(self, timeout=1.0) -> bool:
        """제어 명령이 최근 데이터인지 확인"""
        return (time.time() - self.last_cmd_vel_time) < timeout
    
    def is_battery_recent(self, timeout=10.0) -> bool:
        """배터리 데이터가 최근 데이터인지 확인"""
        return (time.time() - self.last_battery_time) < timeout
    
    def get_distance_to_target(self) -> Optional[float]:
        """현재 위치에서 목표까지의 거리 계산"""
        if not self.camera_pose or not self.target_pose:
            return None
        
        current_pos = self.camera_pose.pose.position
        target_pos = self.target_pose.pose.position
        
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        
        return math.sqrt(dx*dx + dy*dy)
    
    def get_current_speed(self) -> Optional[float]:
        """현재 속도 반환"""
        if not self.cmd_vel:
            return None
        
        linear_speed = math.sqrt(
            self.cmd_vel.linear.x**2 + 
            self.cmd_vel.linear.y**2
        )
        return linear_speed
    
    def to_dict(self) -> Dict:
        """딕셔너리 형태로 변환 (JSON 직렬화용)"""
        result = {
            'robot_id': self.robot_id,
            'task': self.task,
            'state': self.state,
            'battery': self.battery,
            'timestamps': {
                'camera_pose': self.last_camera_pose_time,
                'target_pose': self.last_target_pose_time,
                'cmd_vel': self.last_cmd_vel_time,
                'battery': self.last_battery_time,
                'task': self.last_task_time,
                'state': self.last_state_time
            }
        }
        
        # 위치 정보 추가
        if self.camera_pose:
            pos = self.camera_pose.pose.position
            result['current_position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
        
        if self.target_pose:
            pos = self.target_pose.pose.position
            result['target_position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
        
        # 속도 정보 추가
        if self.cmd_vel:
            result['velocity'] = {
                'linear': {'x': self.cmd_vel.linear.x, 'y': self.cmd_vel.linear.y},
                'angular': {'z': self.cmd_vel.angular.z}
            }
        
        # 계산된 정보 추가
        distance = self.get_distance_to_target()
        if distance is not None:
            result['distance_to_target'] = distance
        
        speed = self.get_current_speed()
        if speed is not None:
            result['current_speed'] = speed
        
        return result


class RobotStatusSubscriber(Node):
    """로봇 상태 구독자 노드"""
    
    def __init__(self, robot_ids: List[int] = None):
        super().__init__('robot_status_subscriber')
        
        # 기본 로봇 ID 설정
        if robot_ids is None:
            robot_ids = [1, 2, 3, 4, 5]
        
        self.robot_ids = robot_ids
        self.robot_statuses: Dict[int, RobotStatus] = {}
        
        # 각 로봇에 대한 상태 객체 초기화
        for robot_id in self.robot_ids:
            self.robot_statuses[robot_id] = RobotStatus(robot_id=robot_id)
        
        # 구독자 설정
        self.setup_subscribers()
        
        # 상태 모니터링 타이머 (1Hz)
        self.status_timer = self.create_timer(1.0, self.log_robot_statuses)
        
        # JSON 상태 발행 타이머 (0.5Hz) - 필요시 활성화
        self.json_publisher = self.create_publisher(String, '/robot_statuses', 10)
        self.json_timer = self.create_timer(2.0, self.publish_json_status)
        
        self.get_logger().info(f'🤖 Robot Status Subscriber initialized for robots: {robot_ids}')
    
    def setup_subscribers(self):
        """모든 로봇에 대한 구독자 설정"""
        for robot_id in self.robot_ids:
            self.setup_robot_subscribers(robot_id)
    
    def setup_robot_subscribers(self, robot_id: int):
        """개별 로봇에 대한 구독자 설정"""
        
        # Camera Pose 구독
        self.create_subscription(
            PoseStamped,
            f'/robot{robot_id}/camera_pose',
            lambda msg, rid=robot_id: self.camera_pose_callback(msg, rid),
            10
        )
        
        # Target Pose 구독
        self.create_subscription(
            PoseStamped,
            f'/robot{robot_id}/target_pose',
            lambda msg, rid=robot_id: self.target_pose_callback(msg, rid),
            10
        )
        
        # CMD Vel 구독
        self.create_subscription(
            Twist,
            f'/robot{robot_id}/cmd_vel',
            lambda msg, rid=robot_id: self.cmd_vel_callback(msg, rid),
            10
        )
        
        # Battery 구독
        self.create_subscription(
            Float32,
            f'/robot{robot_id}/battery',
            lambda msg, rid=robot_id: self.battery_callback(msg, rid),
            10
        )
        
        # Task 구독 (String 타입으로 가정)
        self.create_subscription(
            String,
            f'/robot{robot_id}/task',
            lambda msg, rid=robot_id: self.task_callback(msg, rid),
            10
        )
        
        # State 구독 (String 타입으로 가정)
        self.create_subscription(
            String,
            f'/robot{robot_id}/state',
            lambda msg, rid=robot_id: self.state_callback(msg, rid),
            10
        )
        
        # Joint States 구독 (로봇팔용)
        self.create_subscription(
            JointState,
            f'/robot{robot_id}/joint_states',
            lambda msg, rid=robot_id: self.joint_states_callback(msg, rid),
            10
        )
        
        self.get_logger().info(f'📡 Subscribed to all topics for robot {robot_id}')
    
    def camera_pose_callback(self, msg: PoseStamped, robot_id: int):
        """카메라 포즈 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].camera_pose = msg
            self.robot_statuses[robot_id].last_camera_pose_time = time.time()
    
    def target_pose_callback(self, msg: PoseStamped, robot_id: int):
        """타겟 포즈 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].target_pose = msg
            self.robot_statuses[robot_id].last_target_pose_time = time.time()
            
            pos = msg.pose.position
            self.get_logger().debug(f'🎯 Robot {robot_id} target updated: ({pos.x:.2f}, {pos.y:.2f})')
    
    def cmd_vel_callback(self, msg: Twist, robot_id: int):
        """속도 명령 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].cmd_vel = msg
            self.robot_statuses[robot_id].last_cmd_vel_time = time.time()
    
    def battery_callback(self, msg: Float32, robot_id: int):
        """배터리 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].battery = msg.data
            self.robot_statuses[robot_id].last_battery_time = time.time()
            
            # 배터리 부족 경고
            if msg.data < 0.2:  # 20% 미만
                self.get_logger().warn(f'🔋 Robot {robot_id} LOW BATTERY: {msg.data*100:.1f}%')
    
    def task_callback(self, msg: String, robot_id: int):
        """작업 상태 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].task = msg.data
            self.robot_statuses[robot_id].last_task_time = time.time()
            
            self.get_logger().info(f'📋 Robot {robot_id} task: {msg.data}')
    
    def state_callback(self, msg: String, robot_id: int):
        """로봇 상태 콜백"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].state = msg.data
            self.robot_statuses[robot_id].last_state_time = time.time()
            
            self.get_logger().info(f'🤖 Robot {robot_id} state: {msg.data}')
    
    def joint_states_callback(self, msg: JointState, robot_id: int):
        """관절 상태 콜백 (로봇팔용)"""
        if robot_id in self.robot_statuses:
            self.robot_statuses[robot_id].joint_states = msg
            self.robot_statuses[robot_id].last_joint_states_time = time.time()
    
    def get_robot_status(self, robot_id: int) -> Optional[RobotStatus]:
        """특정 로봇의 상태 반환"""
        return self.robot_statuses.get(robot_id)
    
    def get_all_robot_statuses(self) -> Dict[int, RobotStatus]:
        """모든 로봇 상태 반환"""
        return self.robot_statuses.copy()
    
    def is_robot_online(self, robot_id: int) -> bool:
        """로봇이 온라인 상태인지 확인"""
        if robot_id not in self.robot_statuses:
            return False
        
        status = self.robot_statuses[robot_id]
        # 카메라 포즈나 배터리 정보가 최근 데이터면 온라인으로 판단
        return (status.is_camera_pose_recent() or 
                status.is_battery_recent())
    
    def get_online_robots(self) -> List[int]:
        """온라인 상태인 로봇 ID 리스트 반환"""
        return [robot_id for robot_id in self.robot_ids 
                if self.is_robot_online(robot_id)]
    
    def log_robot_statuses(self):
        """로봇 상태 로그 출력"""
        online_robots = self.get_online_robots()
        
        if not online_robots:
            self.get_logger().warn('⚠️ No robots online')
            return
        
        for robot_id in online_robots:
            status = self.robot_statuses[robot_id]
            
            # 기본 정보
            status_msg = f'🤖 Robot{robot_id} |'
            
            # 상태 및 작업
            if status.state:
                status_msg += f' State: {status.state} |'
            if status.task:
                status_msg += f' Task: {status.task} |'
            
            # 배터리
            if status.battery is not None:
                battery_icon = '🔋' if status.battery > 0.5 else '🪫' if status.battery > 0.2 else '⚠️'
                status_msg += f' {battery_icon} {status.battery*100:.1f}% |'
            
            # 위치 정보
            if status.camera_pose:
                pos = status.camera_pose.pose.position
                status_msg += f' Pos: ({pos.x:.2f},{pos.y:.2f}) |'
            
            # 목표 및 거리
            distance = status.get_distance_to_target()
            if distance is not None:
                status_msg += f' Target Dist: {distance:.2f}m |'
            
            # 속도
            speed = status.get_current_speed()
            if speed is not None and speed > 0.01:
                status_msg += f' Speed: {speed:.2f}m/s |'
            
            # 데이터 연결 상태
            connectivity = []
            if status.is_camera_pose_recent():
                connectivity.append('📍')
            if status.is_target_pose_recent():
                connectivity.append('🎯')
            if status.is_cmd_vel_recent():
                connectivity.append('🎮')
            
            if connectivity:
                status_msg += f' [{" ".join(connectivity)}]'
            
            self.get_logger().info(status_msg)
    
    def publish_json_status(self):
        """JSON 형태로 모든 로봇 상태 발행"""
        try:
            all_statuses = {}
            for robot_id, status in self.robot_statuses.items():
                if self.is_robot_online(robot_id):
                    all_statuses[str(robot_id)] = status.to_dict()
            
            if all_statuses:
                json_msg = String()
                json_msg.data = json.dumps(all_statuses, indent=2)
                self.json_publisher.publish(json_msg)
                
        except Exception as e:
            self.get_logger().error(f'JSON status publication failed: {e}')
    
    def get_summary_report(self) -> str:
        """요약 리포트 생성"""
        online_robots = self.get_online_robots()
        total_robots = len(self.robot_ids)
        
        report = f"📊 Robot Status Summary\n"
        report += f"{'='*50}\n"
        report += f"Total Robots: {total_robots}\n"
        report += f"Online Robots: {len(online_robots)} ({online_robots})\n"
        report += f"Offline Robots: {total_robots - len(online_robots)}\n"
        report += f"{'='*50}\n"
        
        for robot_id in sorted(online_robots):
            status = self.robot_statuses[robot_id]
            report += f"\nRobot {robot_id}:\n"
            report += f"  State: {status.state or 'Unknown'}\n"
            report += f"  Task: {status.task or 'None'}\n"
            report += f"  Battery: {status.battery*100:.1f}% " if status.battery else "  Battery: Unknown "
            
            if status.battery and status.battery < 0.3:
                report += "⚠️ LOW\n"
            else:
                report += "\n"
                
            if status.camera_pose and status.target_pose:
                distance = status.get_distance_to_target()
                report += f"  Target Distance: {distance:.2f}m\n" if distance else ""
            
            speed = status.get_current_speed()
            if speed and speed > 0.01:
                report += f"  Current Speed: {speed:.2f}m/s\n"
        
        return report


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    # 로봇 ID 리스트 설정 (필요에 따라 수정)
    robot_ids = [1, 2, 3, 4, 5]
    
    # 커맨드라인 인자로 로봇 ID 지정 가능
    import sys
    if len(sys.argv) > 1:
        try:
            robot_ids = [int(x) for x in sys.argv[1].split(',')]
        except ValueError:
            print("Invalid robot IDs. Using default: [1, 2, 3, 4, 5]")
    
    print(f"🚀 Starting Robot Status Subscriber for robots: {robot_ids}")
    
    try:
        subscriber = RobotStatusSubscriber(robot_ids)
        
        # 주기적으로 요약 리포트 출력 (30초마다)
        def print_summary():
            print("\n" + subscriber.get_summary_report())
        
        # 타이머로 요약 리포트 출력
        summary_timer = subscriber.create_timer(30.0, print_summary)
        
        rclpy.spin(subscriber)
        
    except KeyboardInterrupt:
        print(f"\n⏹️ Robot Status Subscriber stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'subscriber' in locals():
            subscriber.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()