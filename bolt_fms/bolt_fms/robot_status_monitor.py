import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from enum import Enum


class RobotStatus(Enum):
    IDLE = "Idle"
    BUSY = "Busy"
    ERROR = "Error"


ROBOT_INFO = {
    1: {'type': 'mobile', 'status': RobotStatus.IDLE},
    2: {'type': 'mobile', 'status': RobotStatus.IDLE},
    3: {'type': 'mobile', 'status': RobotStatus.IDLE},
    4: {'type': 'robot_arm', 'status': RobotStatus.IDLE},
    5: {'type': 'robot_arm', 'status': RobotStatus.IDLE},
}


class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__('robot_state_monitor_node')

        self.robot_states = {}  # {robot_id: {"battery": ..., "status": ..., "type": ...}}
        
        self.timer = self.create_timer(1.0, self.check_all_status)

        # 초기화
        for robot_id, info in ROBOT_INFO.items():
            self.robot_states[robot_id] = {
                "type": info['type'],
                "status": info['status'],
                "battery": None,
                "pose": None,
                "now_task": None,
            }

            topic_battery = f'/robot{robot_id}/battery'
            self.create_subscription(
                BatteryState,
                topic_battery,
                self.make_battery_callback(robot_id),
                10
            )
            self.get_logger().info(f"🔌 Subscribed to {topic_battery} from robot{robot_id}")

            topic_pose = f'/robot{robot_id}/camera_pose'
            self.create_subscription(
                PoseStamped,
                topic_pose,
                self.make_pose_callback(robot_id),
                10
            )
            self.get_logger().info(f"🔌 Subscribed to {topic_pose} from robot{robot_id}")

    def make_battery_callback(self, robot_id):
        def callback(msg: BatteryState):
            self.robot_states[robot_id]["battery"] = msg.percentage
            self.get_logger().info(
                f"[로봇 {robot_id}] 🔋 배터리: {msg.percentage * 100:.1f}%"
            )
        return callback

    def make_pose_callback(self, robot_id):
        def callback(msg: PoseStamped):
            self.robot_states[robot_id]["pose"] = msg.pose
            self.get_logger().info(
                f"[로봇 {robot_id}] 📍 위치: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
            )
        return callback

    def check_all_status(self):
        for robot_id, info in self.robot_states.items():
            status = info['status']
            if status == RobotStatus.IDLE:
                self.get_logger().info(f"🤖 Robot {robot_id} is idle.")
            elif status == RobotStatus.BUSY:
                self.get_logger().info(f"🚧 Robot {robot_id} is busy.")
            elif status == RobotStatus.ERROR:
                self.get_logger().warn(f"❗ Robot {robot_id} has an error.")
            else:
                self.get_logger().warn(f"❓ Unknown status for Robot {robot_id}")

    

    

def main(args=None):
    rclpy.init(args=args)
    monitor = RobotStateMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()