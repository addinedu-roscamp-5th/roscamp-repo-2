#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SafetyBubbleNode(Node):
    def __init__(self):
        super().__init__('safety_bubble_node')
        self.declare_parameter('bubble_radius', 0.1)
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.prev_stop = False
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.stop_pub = self.create_publisher(Bool, '/cmd_stop', 10)
        self.get_logger().info(f'Safety bubble active: radius={self.bubble_radius*100:.0f}cm')

    def scan_callback(self, msg):
        min_dist = min((r for r in msg.ranges if r==r), default=float('inf'))
        stop = (min_dist <= self.bubble_radius)
        if stop and not self.prev_stop:
            self.get_logger().warn(f'Obstacle entered bubble: {min_dist:.2f}m')
        elif not stop and self.prev_stop:
            self.get_logger().info(f'Cleared bubble: {min_dist:.2f}m')
        self.prev_stop = stop
        self.stop_pub.publish(Bool(data=stop))
        if not stop:
            self.get_logger().debug(f'No obstacle (min_dist={min_dist:.2f}m)')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBubbleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()