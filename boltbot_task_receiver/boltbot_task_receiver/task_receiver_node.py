#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskReceiverNode(Node):
    """
    관제(FMS)로부터 'task_type'을 구독하여 로봇의 전반적인 상태를 제어하고,
    'drive_state'를 발행하여 주행 가능 여부를 상태 머신에 알리는 노드.
    """
    def __init__(self):
        super().__init__('task_receiver_node')

        # 도메인 브리지를 통해 리매핑된 'task_type' 토픽을 구독
        task_type_topic = 'task_type'
        
        self.task_subscription = self.create_subscription(
            String,
            task_type_topic,
            self.task_type_callback,
            10)
            
        # RobotGoalController(상태 머신)로 주행 상태를 전달할 퍼블리셔
        self.drive_state_publisher = self.create_publisher(
            String,
            'drive_state', 
            10)

        self.get_logger().info('Task Receiver Node has been started.')
        self.get_logger().info(f'Listening for task types on topic: "{task_type_topic}"')

    def task_type_callback(self, msg):
        """FMS로부터 새로운 task_type을 받았을 때 호출되는 콜백 함수"""
        task_type = msg.data
        drive_state_msg = String()
        
        self.get_logger().info(f'New task type received: "{task_type}"')
        
        if task_type == 'MOVE':
            # 주행 명령이 들어오면, 상태 머신이 움직일 수 있도록 'RUN' 상태를 발행
            self.get_logger().info('-> Task is MOVE. Setting drive_state to RUN.')
            drive_state_msg.data = "RUN"
            self.drive_state_publisher.publish(drive_state_msg)
            
        elif task_type == 'IDLE':
            # IDLE 명령이 들어오면, 상태 머신이 멈추도록 'STOP' 상태를 발행
            self.get_logger().info('-> Task is IDLE. Setting drive_state to STOP.')
            drive_state_msg.data = "STOP"
            self.drive_state_publisher.publish(drive_state_msg)
            
        elif task_type == 'WAIT_USER':
            self.get_logger().info('-> Task is WAIT_USER.')
            # TODO: LED 점등 등 사용자 대기 상태에 대한 동작 수행
            pass
            
        else:
            self.get_logger().warn(f'-> Unknown task type: "{task_type}"')

def main(args=None):
    rclpy.init(args=args)
    node = TaskReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()