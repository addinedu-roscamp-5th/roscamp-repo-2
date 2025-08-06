#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from ament_index_python.packages import get_package_share_directory
import socket
import struct
import os
import cv2
import numpy as np
from ultralytics import YOLO

class YoloFlowStopNode(Node):
    def __init__(self):
        super().__init__('yolo_flow_stop_node')

        # ─── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('port', 9991)
        self.declare_parameter('flow_thresh', 2.0)
        self.declare_parameter('pinky_thr', 0.27)
        self.declare_parameter('worker_thr', 0.19)

        self.port        = self.get_parameter('port').value
        self.flow_thresh = self.get_parameter('flow_thresh').value
        self.thr = {
            'pinky':  self.get_parameter('pinky_thr').value,
            'worker': self.get_parameter('worker_thr').value,
        }

        # ─── 퍼블리셔 ─────────────────────────────────────────────
        self.stop_pub       = self.create_publisher(Bool,    '/frame_stop',            10)
        self.dynamic_pub    = self.create_publisher(Bool,    '/obstacle/dynamic',      10)
        self.box_center_pub = self.create_publisher(Float32, '/obstacle/box_center_x', 10)

        # ─── YOLO 모델 로드 ───────────────────────────────────────
        pkg = get_package_share_directory('boltbot_obstacle')
        model_path = os.path.join(pkg, 'models', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from: {model_path}")

        # ─── 소켓 설정 ───────────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('', self.port))
        self.sock.listen(1)
        self.get_logger().info(f"Listening on port {self.port}")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"Client connected: {addr}")

        # ─── optical flow 초기화 ───────────────────────────────────
        self.prev_gray = None

        # ─── 디버그 윈도우 ─────────────────────────────────────────
        cv2.namedWindow('debug', cv2.WINDOW_NORMAL)

    def receive_frame(self):
        hdr = self.conn.recv(4)
        if not hdr:
            return None
        length = struct.unpack('>L', hdr)[0]
        data = b''
        while len(data) < length:
            packet = self.conn.recv(length - len(data))
            if not packet:
                return None
            data += packet
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def spin_once(self):
        frame = self.receive_frame()
        if frame is None:
            return False

        h, w = frame.shape[:2]
        frame_area = h * w

        # 1) YOLO 객체 감지 + confidence 필터(>= 0.5)
        results = self.model.predict(source=frame, verbose=False)[0]
        raw_boxes = results.boxes.xyxy.tolist()
        raw_cls   = results.boxes.cls.tolist()
        raw_conf  = results.boxes.conf.tolist()

        boxes = []
        cls_ids = []
        for box, cid, conf in zip(raw_boxes, raw_cls, raw_conf):
            if conf >= 0.5:
                boxes.append(box)
                cls_ids.append(cid)

        # 2) 클래스별 최대 면적 비율 계산
        best = {'pinky': 0.0, 'worker': 0.0}
        for cid, (x1, y1, x2, y2) in zip(cls_ids, boxes):
            name = self.model.names[int(cid)]
            if name in best:
                area  = (x2 - x1) * (y2 - y1)
                ratio = area / frame_area
                best[name] = max(best[name], ratio)

        # 3) 정지 여부 결정
        stop = any(best[cls] >= self.thr[cls] for cls in best)
        self.stop_pub.publish(Bool(data=stop))

        # 4) optical flow 로 동적/정적 판단
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if stop and self.prev_gray is not None:
            flow_mag = float(cv2.absdiff(gray, self.prev_gray).mean())
            is_dynamic = (flow_mag >= self.flow_thresh)
        else:
            is_dynamic = False
        self.prev_gray = gray
        self.dynamic_pub.publish(Bool(data=is_dynamic))

        # 5) 바운딩 박스 중심점 퍼블리시 (정적 장애물인 경우에만)
        if stop and not is_dynamic and boxes:
            # 가장 큰 비율을 보인 박스의 중심을 찾아서 퍼블리시
            x1, y1, x2, y2 = max(
                [(box, cid) for cid, box in zip(cls_ids, boxes)
                 if self.model.names[int(cid)] in best],
                key=lambda bc: (bc[0][2] - bc[0][0]) * (bc[0][3] - bc[0][1])
            )[0]
            cx = (x1 + x2) / 2.0
            norm_cx = cx / w  # 0.0(왼쪽) ~ 1.0(오른쪽)
            self.box_center_pub.publish(Float32(data=norm_cx))
        else:
            # 장애물 없거나 동적이거나 박스가 없으면 음수로 표시
            self.box_center_pub.publish(Float32(data=-1.0))

        # 6) 디버그 화면 출력
        dbg = frame.copy()
        for (x1, y1, x2, y2) in boxes:
            cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        txt = (f"pinky {best['pinky']:.2f}/{self.thr['pinky']:.2f}  "
               f"worker {best['worker']:.2f}/{self.thr['worker']:.2f}  "
               f"stop={stop} dyn={is_dynamic}")
        cv2.putText(dbg, txt, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow('debug', dbg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False

        return True

    def run(self):
        try:
            while rclpy.ok() and self.spin_once():
                pass
        except KeyboardInterrupt:
            pass
        finally:
            self.conn.close()
            self.sock.close()
            cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = YoloFlowStopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
