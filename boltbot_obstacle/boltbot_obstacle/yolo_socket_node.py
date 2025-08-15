#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import socket
import struct
import threading
import queue
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String  # ← String 추가
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

try:
    import torch
    HAS_TORCH = True
except Exception:
    HAS_TORCH = False


class MultiYoloUdpNode(Node):
    """
    여러 로봇(포트)에서 UDP 프레임을 비동기 수신 → YOLO 감지 → 로봇별 '정지' 및 '방향' 토픽 발행.
    - stop: 면적비 기반 + 디바운스/히스테리시스
    - direction: stop 상태에서 ROI(최대 박스) 내 광류(LK)로 좌/우 추정 후 LEFT/RIGHT 발행
    - 옵션: 특정 포트(또는 최신 프레임)의 결과를 공통 토픽(/frame_stop, /direction)으로 미러.
    """
    def __init__(self):
        super().__init__('multi_yolo_udp_node')

        # ── 파라미터: 기본 인식/임계 ───────────────────────────────
        self.declare_parameter('ports', [9991, 9992, 9993])
        self.declare_parameter('pinky_thr', 0.27)        # 클래스별 면적 비율 임계
        self.declare_parameter('worker_thr', 0.19)
        self.declare_parameter('conf_thresh', 0.5)       # YOLO confidence 임계값

        # stop 디바운스
        self.declare_parameter('stop_on_n', 3)           # 연속 True N회 → stop 전환
        self.declare_parameter('clear_on_n', 3)          # 연속 False N회 → clear 전환
        self.declare_parameter('publish_every', True)    # True면 매 루프 발행, False면 변경시에만

        # 방향(광류) 설정
        self.declare_parameter('flow_dir_thresh', 0.8)   # median dx(px) 절대값이 이 값 이상일 때 방향 유효
        self.declare_parameter('flow_max_corners', 80)
        self.declare_parameter('flow_quality', 0.01)
        self.declare_parameter('flow_min_dist', 7)
        self.declare_parameter('flow_win_size', 15)
        self.declare_parameter('flow_min_points', 10)    # 유효 점 최소 개수
        # 방향 디바운스(좌/우 전환 방지)
        self.declare_parameter('dir_on_n', 2)            # 동일 방향 후보 N프레임 연속 → 확정
        self.declare_parameter('dir_clear_n', 3)         # 방향 확정 후 미검출/약한 신호 N프레임 → 해제
        self.declare_parameter('dir_publish_every', True)

        # 디버그/미러 옵션
        self.declare_parameter('debug_view', True)       # OpenCV 디버그 창
        self.declare_parameter('mirror_global', False)   # 공통 토픽으로 미러 발행 여부
        self.declare_parameter('mirror_strategy', 'primary')  # 'primary' | 'latest'
        self.declare_parameter('mirror_port', 9991)      # primary일 때 기준 포트

        # YOLO 모델 경로 (best.pt)
        self.declare_parameter('model_package', 'boltbot_obstacle')
        self.declare_parameter('model_relpath', os.path.join('models', 'best.pt'))

        # ── 파라미터 로드 ─────────────────────────────────────────
        self.ports = list(self.get_parameter('ports').value)
        self.thr = {
            'pinky':  float(self.get_parameter('pinky_thr').value),
            'worker': float(self.get_parameter('worker_thr').value),
        }
        self.conf_thresh = float(self.get_parameter('conf_thresh').value)

        self.stop_on_n     = int(self.get_parameter('stop_on_n').value)
        self.clear_on_n    = int(self.get_parameter('clear_on_n').value)
        self.publish_every = bool(self.get_parameter('publish_every').value)

        # 광류/방향
        self.flow_dir_thresh   = float(self.get_parameter('flow_dir_thresh').value)
        self.flow_max_corners  = int(self.get_parameter('flow_max_corners').value)
        self.flow_quality      = float(self.get_parameter('flow_quality').value)
        self.flow_min_dist     = int(self.get_parameter('flow_min_dist').value)
        self.flow_win_size     = int(self.get_parameter('flow_win_size').value)
        self.flow_min_points   = int(self.get_parameter('flow_min_points').value)
        self.dir_on_n          = int(self.get_parameter('dir_on_n').value)
        self.dir_clear_n       = int(self.get_parameter('dir_clear_n').value)
        self.dir_publish_every = bool(self.get_parameter('dir_publish_every').value)

        self.debug_view     = bool(self.get_parameter('debug_view').value)
        self.mirror_global  = bool(self.get_parameter('mirror_global').value)
        self.mirror_strategy= str(self.get_parameter('mirror_strategy').value).lower()
        self.mirror_port    = int(self.get_parameter('mirror_port').value)

        # 모델 경로 해석
        model_pkg = str(self.get_parameter('model_package').value)
        model_rel = str(self.get_parameter('model_relpath').value)
        model_path = os.path.join(get_package_share_directory(model_pkg), model_rel)

        # ── YOLO 로드 ─────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        use_half = False
        if HAS_TORCH:
            try:
                use_half = torch.cuda.is_available()
            except Exception:
                use_half = False
        self._yolo_use_half = use_half
        self.get_logger().info(f"YOLO half precision: {self._yolo_use_half}")

        # ── QoS & 퍼블리셔 컨테이너 ──────────────────────────────
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.yolo_publishers = {}   # {port: {'stop': pub, 'direction': pub}}
        # 상태: stop 디바운스 + optical flow용 이전 프레임/방향 디바운스
        # {port: {'filtered': bool, 't_cnt': int, 'f_cnt': int,
        #         'prev_gray': np.ndarray or None,
        #         'dir': '', 'left_cnt': int, 'right_cnt': int, 'none_cnt': int}}
        self.state = {}

        # 공통 토픽(옵션)
        self.global_pub_stop = None
        self.global_pub_dir  = None
        if self.mirror_global:
            self.global_pub_stop = self.create_publisher(Bool,   '/frame_stop',  qos)
            self.global_pub_dir  = self.create_publisher(String, '/direction',   qos)

        # ── 공유 큐 & 소켓 스레드 ─────────────────────────────────
        self.frame_queue = queue.Queue(maxsize=60)
        self.sockets = {}

        for port in self.ports:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                sock.settimeout(0.5)
                sock.bind(('', int(port)))
                self.sockets[port] = sock
                threading.Thread(target=self._udp_receiver_thread, args=(port,), daemon=True).start()
                self.get_logger().info(f"UDP receiver started on port {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to bind UDP socket on port {port}: {e}")

        self._latest_port = None
        self.get_logger().info("MultiYoloUdpNode (stop+direction) is ready.")

    # ─────────────────────────────────────────────────────────────
    # 로봇(포트)별 퍼블리셔/상태 생성
    # ─────────────────────────────────────────────────────────────
    def _get_or_create_robot_context(self, port):
        if port not in self.yolo_publishers:
            prefix = f'/robot_{port}'
            self.yolo_publishers[port] = {
                'stop':      self.create_publisher(Bool,   f'{prefix}/frame_stop', 10),
                'direction': self.create_publisher(String, f'{prefix}/direction',   10),
            }
            if self.debug_view:
                cv2.namedWindow(f'debug_{port}', cv2.WINDOW_NORMAL)
            self.state[port] = {
                'filtered': False, 't_cnt': 0, 'f_cnt': 0,
                'prev_gray': None,
                'dir': '', 'left_cnt': 0, 'right_cnt': 0, 'none_cnt': 0,
                'last_pub_stop': None, 'last_pub_dir': None
            }
            self.get_logger().info(f"New robot context created for port {port}")
        return self.yolo_publishers[port]

    # ─────────────────────────────────────────────────────────────
    # UDP 수신 스레드(프로듀서)
    # ─────────────────────────────────────────────────────────────
    def _udp_receiver_thread(self, port):
        sock = self.sockets.get(port)
        while rclpy.ok():
            try:
                packet, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP receiver error on port {port}: {e}")
                break

            try:
                if len(packet) < 4:
                    continue
                length = struct.unpack('>L', packet[:4])[0]
                data = packet[4:]
                if len(data) < length:
                    continue
                frame = cv2.imdecode(np.frombuffer(data[:length], np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass

                self.frame_queue.put((port, frame))
            except Exception as e:
                self.get_logger().error(f"Packet parse error on port {port}: {e}")
                continue

    # ─────────────────────────────────────────────────────────────
    # Optical Flow 기반 방향 추정 (ROI 내 median dx)
    # ─────────────────────────────────────────────────────────────
    def _estimate_direction(self, prev_gray, gray, roi):
        (x1, y1, x2, y2) = roi
        x1 = max(0, int(x1)); y1 = max(0, int(y1))
        x2 = max(x1 + 1, int(x2)); y2 = max(y1 + 1, int(y2))

        prev_roi = prev_gray[y1:y2, x1:x2]
        curr_roi = gray[y1:y2, x1:x2]

        if prev_roi.size == 0 or curr_roi.size == 0:
            return ''  # 결정 불가

        pts = cv2.goodFeaturesToTrack(
            prev_roi,
            maxCorners=self.flow_max_corners,
            qualityLevel=self.flow_quality,
            minDistance=self.flow_min_dist
        )
        if pts is None or len(pts) < self.flow_min_points:
            return ''

        nxt, st, err = cv2.calcOpticalFlowPyrLK(
            prev_roi, curr_roi, pts, None,
            winSize=(self.flow_win_size, self.flow_win_size),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        if nxt is None or st is None:
            return ''

        good = st.reshape(-1) == 1
        if not np.any(good):
            return ''

        dxy = (nxt - pts)[good].reshape(-1, 2)
        dx = dxy[:, 0]  # ROI 좌표계에서 +dx: 오른쪽
        if dx.size < self.flow_min_points:
            return ''

        med_dx = float(np.median(dx))
        if med_dx <= -self.flow_dir_thresh:
            return 'RIGHT'
        elif med_dx >= self.flow_dir_thresh:
            return 'LEFT'
        return ''  # 움직임 약함/불분명

    # ─────────────────────────────────────────────────────────────
    # 메인 처리 루프(컨슈머)
    # ─────────────────────────────────────────────────────────────
    def main_processing_loop(self):
        while rclpy.ok():
            try:
                port, frame = self.frame_queue.get(timeout=0.5)
                self._latest_port = port
            except queue.Empty:
                continue

            pubs = self._get_or_create_robot_context(port)
            st = self.state[port]

            try:
                h, w = frame.shape[:2]
                frame_area = float(h * w)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # 1) YOLO 추론
                results = self.model.predict(
                    source=frame,
                    verbose=False,
                    half=self._yolo_use_half
                )[0]
                xyxy = results.boxes.xyxy.tolist()
                confs = results.boxes.conf.tolist()
                cls_ids = [int(c) for c in results.boxes.cls.tolist()]

                filt = [(box, cid, cf) for box, cid, cf in zip(xyxy, cls_ids, confs) if cf >= self.conf_thresh]

                # 2) 클래스별 최대 면적 비율
                best = {k: 0.0 for k in self.thr.keys()}
                for (x1, y1, x2, y2), cid, _ in filt:
                    name = self.model.names[cid]
                    if name in best:
                        area_ratio = max(0.0, (x2 - x1) * (y2 - y1)) / max(1.0, frame_area)
                        if area_ratio > best[name]:
                            best[name] = area_ratio

                # 3) raw stop 판단
                raw_stop = any(best[name] >= self.thr[name] for name in best)

                # 4) stop 디바운스 상태 갱신
                if raw_stop:
                    st['t_cnt'] += 1; st['f_cnt'] = 0
                    if (not st['filtered']) and (st['t_cnt'] >= self.stop_on_n):
                        st['filtered'] = True
                else:
                    st['f_cnt'] += 1; st['t_cnt'] = 0
                    if st['filtered'] and (st['f_cnt'] >= self.clear_on_n):
                        st['filtered'] = False
                        # stop 해제 시 방향 상태도 초기화
                        st['dir'] = ''; st['left_cnt'] = st['right_cnt'] = st['none_cnt'] = 0

                filtered_stop = st['filtered']

                # 5) stop 발행
                if self.publish_every:
                    pubs['stop'].publish(Bool(data=filtered_stop))
                else:
                    if filtered_stop != st['last_pub_stop']:
                        pubs['stop'].publish(Bool(data=filtered_stop))
                st['last_pub_stop'] = filtered_stop

                # 6) 방향 추정(정지 상태에서만)
                direction_candidate = ''
                roi_box = None
                if filtered_stop and st['prev_gray'] is not None:
                    # stop에 기여 가능한 가장 큰 박스(관심 클래스) 선택
                    candidate_boxes = []
                    for (x1, y1, x2, y2), cid, _ in filt:
                        name = self.model.names[cid]
                        if name in self.thr:  # 관심 클래스만
                            area = (x2 - x1) * (y2 - y1)
                            candidate_boxes.append(((x1, y1, x2, y2), area))
                    if candidate_boxes:
                        roi_box = max(candidate_boxes, key=lambda t: t[1])[0]
                        direction_candidate = self._estimate_direction(st['prev_gray'], gray, roi_box)

                # 7) 방향 디바운스
                if direction_candidate == 'LEFT':
                    st['left_cnt'] += 1; st['right_cnt'] = 0; st['none_cnt'] = 0
                    if st['dir'] != 'LEFT' and st['left_cnt'] >= self.dir_on_n:
                        st['dir'] = 'LEFT'
                elif direction_candidate == 'RIGHT':
                    st['right_cnt'] += 1; st['left_cnt'] = 0; st['none_cnt'] = 0
                    if st['dir'] != 'RIGHT' and st['right_cnt'] >= self.dir_on_n:
                        st['dir'] = 'RIGHT'
                else:
                    st['none_cnt'] += 1; st['left_cnt'] = 0; st['right_cnt'] = 0
                    if st['dir'] and st['none_cnt'] >= self.dir_clear_n:
                        st['dir'] = ''  # 해제

                # 8) 방향 발행(LEFT/RIGHT만 발행)
                if st['dir']:
                    if self.dir_publish_every or (st['dir'] != st['last_pub_dir']):
                        pubs['direction'].publish(String(data=st['dir']))
                        if self.mirror_global and self.global_pub_dir and self._should_mirror_from(port):
                            self.global_pub_dir.publish(String(data=st['dir']))
                    st['last_pub_dir'] = st['dir']

                # 9) 공통 stop 미러
                if self.mirror_global and self.global_pub_stop and self._should_mirror_from(port):
                    if self.publish_every or (filtered_stop != st['last_pub_stop']):
                        self.global_pub_stop.publish(Bool(data=filtered_stop))

                # 10) 디버그
                if self.debug_view:
                    dbg = frame.copy()
                    for (x1, y1, x2, y2), cid, cf in filt:
                        cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        name = self.model.names[cid]
                        cv2.putText(dbg, f"{name}:{cf:.2f}", (int(x1), int(y1) - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    status = f"PORT[{port}] stop:{filtered_stop} t{st['t_cnt']}/f{st['f_cnt']} dir:{st['dir'] or '-'}"
                    cv2.putText(dbg, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    if roi_box is not None:
                        x1,y1,x2,y2 = map(int, roi_box)
                        cv2.rectangle(dbg, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(dbg, "FLOW ROI", (x1, max(0, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
                    cv2.imshow(f'debug_{port}', dbg)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                # 11) 마지막으로 prev_gray 업데이트
                st['prev_gray'] = gray

            except Exception as e:
                self.get_logger().error(f"Processing error (port {port}): {e}")
            finally:
                self.frame_queue.task_done()

        # 종료 처리
        for sock in self.sockets.values():
            try:
                sock.close()
            except Exception:
                pass
        if self.debug_view:
            cv2.destroyAllWindows()

    # ─────────────────────────────────────────────────────────────
    # 미러 전략 판단
    # ─────────────────────────────────────────────────────────────
    def _should_mirror_from(self, port: int) -> bool:
        if self.mirror_strategy == 'latest':
            return port == self._latest_port
        return port == self.mirror_port


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
