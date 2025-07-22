import cv2
import numpy as np
import time
from pupil_apriltags import Detector

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

cap = cv2.VideoCapture(3)

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    print("오류: 카메라를 열 수 없습니다.")
    exit()

prev_time = time.time()

KNOWN_TAG_SIZE = 0.065
FOCAL_LENGTH = 1000

def estimate_distance(pixel_width, known_width, focal_length):
    if pixel_width == 0:
        return 0
    return (known_width * focal_length) / pixel_width

print("카메라 피드를 시작합니다. 'q'를 눌러 종료하세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임을 읽을 수 없습니다.")
        break

    # FPS 계산
    current_time = time.time()
    fps = 1.0 / (current_time - prev_time)
    prev_time = current_time

    # FPS 텍스트 왼쪽 상단 표시
    fps_text = f"FPS: {fps:.2f}"
    cv2.putText(frame, fps_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

    tag_ids = []
    tag_centers = []
    tag_id_to_center = {}  # tag_id를 키로, 중심 좌표를 값으로 저장할 딕셔너리


    for tag in tags:
        (ptA, ptB, ptC, ptD) = tag.corners
        ptA = (int(ptA[0]), int(ptA[1]))
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))

        # 태그 중심
        cX, cY = int(tag.center[0]), int(tag.center[1])
        tag_centers.append([cX, cY])

        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))

        pixel_width = np.linalg.norm(np.array(ptA) - np.array(ptB))
        distance = estimate_distance(pixel_width, KNOWN_TAG_SIZE, FOCAL_LENGTH)
        # print(pixel_width)
        tag_id = tag.tag_id
        info_text_id = f"ID: {tag_id}"
        info_text_coords = f"XY: ({cX}, {cY})"
        # print(tag_id, cX, cY)
        info_text_dist = f"Z: {distance:.2f} m"

        info_y_offset = cY - 15
        cv2.putText(frame, info_text_id, (cX + 10, info_y_offset - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, info_text_coords, (cX + 10, info_y_offset - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, info_text_dist, (cX + 10, info_y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        axis_length = 50
        center_point = (cX, cY)

        vec_x = np.array(ptB) - np.array(ptA)
        norm_vec_x = vec_x / (np.linalg.norm(vec_x) + 1e-6)
        end_point_x = tuple((np.array(center_point) + norm_vec_x * axis_length).astype(int))
        cv2.arrowedLine(frame, center_point, end_point_x, (0, 0, 255), 3)
        cv2.putText(frame, 'X', (end_point_x[0] + 5, end_point_x[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        vec_y = np.array(ptD) - np.array(ptA)
        norm_vec_y = vec_y / (np.linalg.norm(vec_y) + 1e-6)
        end_point_y = tuple((np.array(center_point) + norm_vec_y * axis_length).astype(int))
        cv2.arrowedLine(frame, center_point, end_point_y, (255, 0, 0), 3)
        cv2.putText(frame, 'Y', (end_point_y[0], end_point_y[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        tag_id = tag.tag_id
        tag_ids.append(tag_id)
        tag_id_to_center[tag_id] = (cX, cY)

    # 태그 중심점 기준으로 볼록 껍질로 외곽선 그리기
    if len(tag_centers) >= 3:
        points = np.array(tag_centers, dtype=np.int32)
        hull = cv2.convexHull(points)
        cv2.polylines(frame, [hull], isClosed=True, color=(255, 0, 255), thickness=3)
            # 태그 중심점 기준으로 볼록 껍질로 외곽선 그리기
    if len(tag_centers) >= 3:
        points = np.array(tag_centers, dtype=np.int32)
        hull = cv2.convexHull(points)

        # 1. 볼록 껍질 그리기
        cv2.polylines(frame, [hull], isClosed=True, color=(255, 0, 255), thickness=3)

        # 2. 마스크 생성
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, hull, 255)

        # 3. 격자 간격 및 크기 설정
        grid_gap = 20  # 픽셀 간격
        grid_color = (0, 255, 255)  # 노란색
        grid_size = 5

        # 4. 마스크 내부 점에 격자 그리기
        min_x = np.min(hull[:, 0, 0])
        max_x = np.max(hull[:, 0, 0])
        min_y = np.min(hull[:, 0, 1])
        max_y = np.max(hull[:, 0, 1])

        for y in range(min_y, max_y, grid_gap):
            for x in range(min_x, max_x, grid_gap):
                if mask[y, x] == 255:
                    # 격자점 찍기 (원 or 사각형 선택 가능)
                    # cv2.circle(frame, (x, y), grid_size, grid_color, -1)
                    # 또는 사각형:
                    cv2.rectangle(frame, (x - grid_size, y - grid_size), (x + grid_size, y + grid_size), grid_color, -1)


    # if tag_ids:
    #     print("검출된 태그 ID:", tag_ids)

    cv2.imshow("AprilTag Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(tag_id_to_center.get(2))
        break

cap.release()
cv2.destroyAllWindows()

print("스크립트를 종료합니다.")