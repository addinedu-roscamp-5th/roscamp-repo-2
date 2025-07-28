import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

def main():
    # rclpy 초기화
    # ROS2 파이썬 클라이언트 라이브러리를 초기화합니다.
    rclpy.init()
    # 'target_pose_publisher'라는 이름의 ROS2 노드 생성
    # 노드는 ROS2 네트워크에서 통신의 기본 단위입니다.
    node = rclpy.create_node('target_pose_publisher')

    # PoseStamped 메시지를 '/target_pose' 토픽으로 발행하는 퍼블리셔 생성
    # qos 프로파일은 10으로 설정하여 메시지 발행의 신뢰성을 보장합니다.
    goal_publisher = node.create_publisher(PoseStamped, '/target_pose', 10)
    
    # PoseStamped 메시지 객체 생성
    # 목표 지점의 위치와 방향 정보를 담는 메시지입니다.
    goal_pose = PoseStamped()
    # 메시지의 헤더에 프레임 ID를 'map'으로 설정
    # 'map' 프레임은 로봇이 동작하는 환경의 고정된 좌표계를 의미합니다.
    goal_pose.header.frame_id = 'map'
    # z, w 값은 고정
    # 2D 환경에서 로봇의 방향을 나타내는 쿼터니언 값입니다.
    goal_pose.pose.orientation.z = 0.707
    goal_pose.pose.orientation.w = 0.707

    # 루프를 돌며 사용자로부터 x, y 좌표를 입력받아 목표 지점을 발행
    while rclpy.ok():
        try:
            # 사용자로부터 x, y 좌표를 입력받음
            # 입력 형식은 "x y" (예: "1.0 1.1")
            input_str = input("Enter x and y coordinates separated by a space (or 'q' to quit): ")
            
            # 'q'를 입력하면 루프를 종료
            if input_str.lower() == 'q':
                break

            # 입력받은 문자열을 공백으로 분리하여 x와 y 좌표를 추출
            x_str, y_str = input_str.split()
            x = float(x_str)
            y = float(y_str)

            # 메시지의 헤더에 현재 시간을 타임스탬프로 설정
            # 메시지가 생성된 시간을 기록합니다.
            goal_pose.header.stamp = node.get_clock().now().to_msg()
            # 입력받은 x, y 좌표를 메시지의 위치 정보로 설정
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y

            # 완성된 PoseStamped 메시지를 발행
            goal_publisher.publish(goal_pose)
            
            # 발행된 메시지의 타임스탬프와 함께 정보 로그를 출력
            print(f"Published goal: x={x}, y={y} with timestamp: {goal_pose.header.stamp}")
            node.get_logger().info('📤 target_pose published!')

        except ValueError:
            # 잘못된 형식의 입력이 들어왔을 때 에러 메시지 출력
            node.get_logger().error("Invalid input. Please enter two numbers separated by a space.")
        except Exception as e:
            # 기타 예외 발생 시 에러 메시지 출력
            node.get_logger().error(f"An error occurred: {e}")

    # 노드 소멸 및 rclpy 종료
    # 사용이 끝난 노드를 정리하고 rclpy를 종료합니다.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
