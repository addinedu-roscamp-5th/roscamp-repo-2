
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QObject, Signal, Slot
import json
import threading

from robot_status_monitor_ui import RobotStatusMonitor

class RosSubscriber(Node):
    def __init__(self, qt_app_handler):
        super().__init__('robot_status_monitor_subscriber')
        self.qt_app_handler = qt_app_handler
        self.subscription = self.create_subscription(
            String,
            '/robot_statuses',
            self.listener_callback,
            10)
        self.get_logger().info('Listening to /robot_statuses topic...')

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.qt_app_handler.update_signal.emit(data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')

class QtAppHandler(QObject):
    update_signal = Signal(dict)

    def __init__(self, app, monitor):
        super().__init__()
        self.app = app
        self.monitor = monitor
        self.update_signal.connect(self.update_monitor)

    @Slot(dict)
    def update_monitor(self, data):
        self.monitor.update_status(data)

def main(args=None):
    rclpy.init(args=args)

    # Qt App
    app = QApplication(sys.argv)
    monitor = RobotStatusMonitor()
    monitor.show()

    # ROS Node
    qt_app_handler = QtAppHandler(app, monitor)
    ros_subscriber = RosSubscriber(qt_app_handler)

    # Run ROS node in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_subscriber,), daemon=True)
    ros_thread.start()

    # Start Qt event loop
    exit_code = app.exec()

    # Cleanup
    ros_subscriber.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
