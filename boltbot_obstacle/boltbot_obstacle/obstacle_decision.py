#!/usr/bin/env python3
import rclpy
from rclpy.node      import Node
from std_msgs.msg    import Bool

class ObstacleDecisionNode(Node):
    def __init__(self):
        super().__init__('obstacle_decision')
        self.bubble   = False
        self.frame    = False
        self.dynamic  = False

        self.create_subscription(Bool, '/bubble_stop',      self.bubble_cb,  10)
        self.create_subscription(Bool, '/frame_stop',       self.frame_cb,   10)
        self.create_subscription(Bool, '/obstacle/dynamic', self.dynamic_cb, 10)

        self.pub = self.create_publisher(Bool, '/cmd_stop', 10)

    def bubble_cb(self, msg):
        self.bubble = msg.data
        self.evaluate()

    def frame_cb(self, msg):
        self.frame = msg.data
        self.evaluate()

    def dynamic_cb(self, msg):
        # 동적 플래그는 정지 중일 때만
        if self.bubble or self.frame:
            self.dynamic = msg.data
        self.evaluate()

    def evaluate(self):
        # 안전버블 우선
        if self.bubble:
            stop = True
        # 카메라 기준 정지
        elif self.frame:
            stop = self.dynamic
        else:
            stop = False

        self.pub.publish(Bool(data=stop))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
