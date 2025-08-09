#!/usr/bin/env python3
import os
import sys
import socket
import struct
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import threading
import queue

class MultiYoloUdpNode(Node):
    """
    여러 로봇으로부터 UDP 프레임을 비동기적으로 수신하여,
    각 로봇별로 객체 감지 및 ROS 토픽 발행을 수행하는 노드.
    """
    def __init__(self):
        super().__init__('multi_yolo_udp_node')

        # --- 파라미터 선언 ---
        self.declare_parameter('ports', [9991, 9992, 9993])
        self.declare_parameter('flow_thresh', 2.0)
        self.declare_parameter('pinky_thr', 0.27)
        self.declare_parameter('worker_thr', 0.19)

        self.ports = self.get_parameter('ports').value
        self.flow_thresh = self.get_parameter('flow_thresh').value
        self.thr = {
            'pinky': self.get_parameter('pinky_thr').value,
            'worker': self.get_parameter('worker_thr').value,
        }

        # --- 로봇(포트)별 데이터 관리를 위한 딕셔너리 ---
        self.yolo_publishers = {}      # Key: port, Value: {'stop': pub, ...}
        self.robot_states = {}         # Key: port, Value: {'prev_gray': img}

        # --- YOLO 모델 로드 ---
        pkg = get_package_share_directory('boltbot_obstacle')
        model_path = os.path.join(pkg, 'models', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from: {model_path}")

        # --- 공유 작업 큐 생성 ---
        self.frame_queue = queue.Queue(maxsize=60)

        # --- 각 포트별 UDP 소켓 및 수신 스레드 설정 ---
        self.sockets = {}
        for port in self.ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            try:
                sock.bind(('', port))
            except Exception as e:
                self.get_logger().error(f"Failed to bind UDP socket on port {port}: {e}")
                continue
            self.sockets[port] = sock

            thread = threading.Thread(target=self._udp_receiver_thread, args=(port,), daemon=True)
            thread.start()
            self.get_logger().info(f"UDP receiver thread started for port {port}")

    def _get_or_create_robot_context(self, port):
        """포트 번호에 해당하는 퍼블리셔와 상태 변수를 가져오거나 새로 생성"""
        if port not in self.yolo_publishers:
            self.get_logger().info(f"New robot on port [{port}] detected. Creating resources.")
            prefix = f'/robot_{port}'
            self.yolo_publishers[port] = {
                'stop':       self.create_publisher(Bool,     f'{prefix}/frame_stop',       10),
                'dynamic':    self.create_publisher(Bool,     f'{prefix}/obstacle/dynamic', 10),
                'box_center': self.create_publisher(Float32,  f'{prefix}/obstacle/box_center_x', 10),
            }
            self.robot_states[port] = {'prev_gray': None}
            cv2.namedWindow(f'debug_{port}', cv2.WINDOW_NORMAL)

        return self.yolo_publishers[port], self.robot_states[port]

    def _udp_receiver_thread(self, port):
        """[프로듀서] 각 포트에서 UDP 패킷을 수신하여 큐에 넣는 스레드 함수"""
        sock = self.sockets.get(port)
        while rclpy.ok():
            try:
                packet, addr = sock.recvfrom(65536)
                if len(packet) < 4:
                    continue
                length = struct.unpack('>L', packet[:4])[0]
                data = packet[4:]
                if len(data) < length:
                    continue
                frame = cv2.imdecode(np.frombuffer(data[:length], np.uint8), cv2.IMREAD_COLOR)
                if frame is not None and not self.frame_queue.full():
                    self.frame_queue.put((port, frame))
            except Exception as e:
                self.get_logger().error(f"UDP receiver error on port {port}: {e}")
                break

    def main_processing_loop(self):
        """[컨슈머] 큐에서 프레임을 꺼내와서 모든 연산을 처리하는 메인 루프"""
        while rclpy.ok():
            try:
                port, frame = self.frame_queue.get(timeout=1.0)
                pubs, state = self._get_or_create_robot_context(port)
                h, w = frame.shape[:2]
                frame_area = h * w

                # 1) YOLO 객체 감지
                results = self.model.predict(source=frame, verbose=False, half=True)[0]
                boxes = [b for b, conf in zip(results.boxes.xyxy.tolist(), results.boxes.conf.tolist()) if conf >= 0.5]
                cls_ids = [int(c) for c, conf in zip(results.boxes.cls.tolist(), results.boxes.conf.tolist()) if conf >= 0.5]

                # 2) 클래스별 최대 면적 비율 계산
                best = {k: 0.0 for k in self.thr.keys()}
                for cid, (x1, y1, x2, y2) in zip(cls_ids, boxes):
                    name = self.model.names[cid]
                    if name in best:
                        area_ratio = ((x2 - x1) * (y2 - y1)) / frame_area
                        best[name] = max(best[name], area_ratio)

                # 3) 정지 판단
                stop = any(best[name] >= self.thr[name] for name in best)
                pubs['stop'].publish(Bool(data=stop))

                # 4) Optical Flow 판단
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                prev_gray = state.get('prev_gray')
                if stop and prev_gray is not None:
                    flow_mag = float(cv2.absdiff(gray, prev_gray).mean())
                    is_dynamic = (flow_mag >= self.flow_thresh)
                else:
                    is_dynamic = False
                state['prev_gray'] = gray
                pubs['dynamic'].publish(Bool(data=is_dynamic))

                # 5) 박스 중심점 퍼블리시
                if stop and not is_dynamic and boxes:
                    x1, y1, x2, y2 = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
                    norm_cx = ((x1 + x2) / 2.0) / w
                    pubs['box_center'].publish(Float32(data=norm_cx))
                else:
                    pubs['box_center'].publish(Float32(data=-1.0))

                # 6) 디버그 화면
                dbg = frame.copy()
                for (x1, y1, x2, y2) in boxes:
                    cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                status = f"PORT[{port}] pinky:{best.get('pinky', 0):.2f} worker:{best.get('worker', 0):.2f} stop:{stop} dyn:{is_dynamic}"
                cv2.putText(dbg, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow(f'debug_{port}', dbg)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                self.frame_queue.task_done()
            except queue.Empty:
                continue

        # 종료 시 자원 정리
        for sock in self.sockets.values():
            sock.close()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = MultiYoloUdpNode()
    try:
        node.main_processing_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down processing loop.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
