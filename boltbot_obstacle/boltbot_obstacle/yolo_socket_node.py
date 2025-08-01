#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import socket
import struct
import os
import cv2
import numpy as np
from ultralytics import YOLO

class YoloSocketNode(Node):
    def __init__(self):
        super().__init__('yolo_socket_node')
        # 포트 파라미터 선언
        self.declare_parameter('port', 9992)
        port = self.get_parameter('port').value

        # ─── 1) 패키지 공유 디렉터리에서 모델 로드 ─────────────────────────
        pkg_share = get_package_share_directory('boltbot_obstacle')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')
        if not os.path.isfile(model_path):
            self.get_logger().error(f'Model not found: {model_path}')
            raise FileNotFoundError(model_path)
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model: {model_path}')

        # ─── 2) 퍼블리셔 & 소켓 초기화 ────────────────────────────────────
        self.pub = self.create_publisher(String, '/yolo/detections', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('', port))
        self.sock.listen(1)
        self.get_logger().info(f'Listening on port {port}')
        self.accept_conn()

    def accept_conn(self):
        conn, addr = self.sock.accept()
        self.get_logger().info(f'Connected: {addr}')
        self.conn = conn

    def spin_once(self):
        data_len_p = self.conn.recv(4)
        if not data_len_p:
            return False
        length = struct.unpack('>L', data_len_p)[0]
        data = b''
        while len(data) < length:
            packet = self.conn.recv(length - len(data))
            if not packet:
                break
            data += packet

        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return True

        results = self.model(frame)
        labels = [f"{r.boxes.cls.tolist()}" for r in results]
        msg = String(data=','.join(labels))
        self.pub.publish(msg)
        return True

    def run(self):
        try:
            while rclpy.ok():
                if not self.spin_once():
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.conn.close()
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = YoloSocketNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
