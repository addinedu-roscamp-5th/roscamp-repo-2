#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory
import socket, struct, os, cv2, numpy as np
from ultralytics import YOLO

class YoloFlowStopNode(Node):
    def __init__(self):
        super().__init__('yolo_flow_stop_node')

        # 파라미터
        self.declare_parameter('port', 9992)
        self.declare_parameter('flow_thresh', 2.0)
        # 클래스별 정지 임계면적 비율
        self.declare_parameter('pinky_thr', 0.27)
        self.declare_parameter('worker_thr',0.19)

        self.port        = self.get_parameter('port').value
        self.flow_thresh = self.get_parameter('flow_thresh').value
        self.thr = {
            'pinky':  self.get_parameter('pinky_thr').value,
            'worker': self.get_parameter('worker_thr').value,
        }

        # 퍼블리셔
        self.stop_pub    = self.create_publisher(Bool,   '/frame_stop',   10)
        self.dynamic_pub = self.create_publisher(Bool,   '/obstacle/dynamic', 10)

        # YOLO 모델
        pkg = get_package_share_directory('boltbot_obstacle')
        model_path = os.path.join(pkg, 'models', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO: {model_path}")

        # 소켓
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('', self.port))
        self.sock.listen(1)
        self.get_logger().info(f"Listening on port {self.port}")
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"Client connected: {addr}")

        # optical flow용 그레이 프레임
        self.prev_gray = None

        # 디버그 윈도우 (선택)
        cv2.namedWindow('debug', cv2.WINDOW_NORMAL)

    def receive_frame(self):
        hdr = self.conn.recv(4)
        if not hdr: return None
        length = struct.unpack('>L', hdr)[0]
        data = b''
        while len(data) < length:
            packet = self.conn.recv(length - len(data))
            if not packet: return None
            data += packet
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def spin_once(self):
        frame = self.receive_frame()
        if frame is None:
            return False

        h, w = frame.shape[:2]
        frame_area = h * w

        # 1) YOLO 감지
        results = self.model.predict(source=frame, verbose=False)[0]
        boxes  = results.boxes.xyxy.tolist()
        cls_ids = results.boxes.cls.tolist()

        # 2) 클래스별 최대 면적비율 계산
        best = {'pinky':0.0, 'worker':0.0}
        for cid, (x1,y1,x2,y2) in zip(cls_ids, boxes):
            name = self.model.names[int(cid)]
            if name in best:
                area  = (x2-x1)*(y2-y1)
                ratio = area / frame_area
                best[name] = max(best[name], ratio)

        # 3) 임계치 비교 → 즉시 정지 토글
        stop = any(best[cls] >= self.thr[cls] for cls in best)
        self.stop_pub.publish(Bool(data=stop))

        # 4) optical flow 판별 (오직 정지 상태에서만)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if stop and self.prev_gray is not None:
            flow_mag = float(cv2.absdiff(gray, self.prev_gray).mean())
            is_dynamic = (flow_mag >= self.flow_thresh)
        else:
            is_dynamic = False

        # 갱신 & 퍼블리시
        self.prev_gray = gray
        self.dynamic_pub.publish(Bool(data=is_dynamic))

        # 5) 디버그 (선택)
        dbg = frame.copy()
        for (x1,y1,x2,y2) in boxes:
            cv2.rectangle(dbg, (int(x1),int(y1)), (int(x2),int(y2)), (0,255,0),2)
        # 상태 텍스트
        txt = f"pinky {best['pinky']:.2f}/{self.thr['pinky']:.2f}  "\
              f"worker {best['worker']:.2f}/{self.thr['worker']:.2f}  "\
              f"stop={stop} dyn={is_dynamic}"
        cv2.putText(dbg, txt, (10,25),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
        cv2.imshow('debug', dbg)
        if cv2.waitKey(1)&0xFF == ord('q'):
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

if __name__=='__main__':
    main()

