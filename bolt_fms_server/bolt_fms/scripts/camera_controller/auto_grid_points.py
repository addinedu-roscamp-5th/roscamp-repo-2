import cv2
import numpy as np
import time
from pupil_apriltags import Detector


ROWS = 12
COLS = 24

REAL_ROWS = ROWS+1
REAL_COLS = COLS+1

REAL_MAX_WIDTH = 2.0
REAL_MAX_HEIGHT = 0.8

REAL_WIDTH = REAL_MAX_WIDTH / REAL_COLS
REAL_HEIGHT = REAL_MAX_HEIGHT / REAL_ROWS

horizontal_divisions = ROWS - 1
vertical_divisions = COLS - 1
printed_once = True  # 태그 ID 한 번만 출력
grid_corners = None   # trapezoid 꼭짓점 저장

# 보간 함수
def interpolate( p1, p2, t):
    return (int(p1[0] + t * (p2[0] - p1[0])), int(p1[1] + t * (p2[1] - p1[1])))

def generate_grid_points( pts, h_div, v_div):
    p0, p1, p2, p3 = pts

    lefts = [interpolate(p0, p2, i / h_div) for i in range(h_div + 1)]
    rights = [interpolate(p1, p3, i / h_div) for i in range(h_div + 1)]

    tops = [interpolate(p0, p1, i / v_div) for i in range(v_div + 1)]
    bottoms = [interpolate(p2, p3, i / v_div) for i in range(v_div + 1)]

    grid_points = []
    for row, (l, r) in enumerate(zip(lefts, rights)):
        for col, (t, b) in enumerate(zip(tops, bottoms)):
            pt = interpolate(l, r, col / v_div)
            grid_points.append((row, col, pt))
    
    return grid_points

def visualize_grid( img, grid_points, rows, cols):

    # 가로줄
    for row in range(rows):
        pts = [pt for r, c, pt in grid_points if r == row]
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (255, 255, 0), 1)

    # 세로줄
    for col in range(cols):
        pts = [pt for r, c, pt in grid_points if c == col]
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], (0, 255, 255), 1)

    # 점
    for row, col, pt in grid_points:
        cv2.circle(img, pt, 2, (0, 0, 255), -1)


# 📍 각 꼭짓점에서 가장 가까운 실제 에이프릴 태그 중심 좌표로 snap
def find_closest( center, candidates):
    min_dist = float('inf')
    closest = center
    for c in candidates:
        dist = np.linalg.norm(np.array(c) - np.array(center))
        if dist < min_dist:
            min_dist = dist
            closest = c
    return tuple(closest)


# AprilTag 감지기 설정
at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("오류: 카메라를 열 수 없습니다.")
    exit()

prev_time = time.time()
print("카메라 피드를 시작합니다. 'q'를 눌러 종료하세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임을 읽을 수 없습니다.")
        break

    h, w = frame.shape[:2]

    top_left     = (0, 0)
    top_right    = (w - 1, 0)
    bottom_left  = (0, h - 1)
    bottom_right = (w - 1, h - 1)

    current_time = time.time()
    fps = 1.0 / (current_time - prev_time)
    prev_time = current_time
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

    tag_centers = []              # [[x, y], ...]
    tag_id_to_center = {}         # {id: (x, y)}
    center_to_tag_id = {}         # {(x, y): id}

    for tag in tags:
        cX, cY = int(tag.center[0]), int(tag.center[1])
        tag_centers.append([cX, cY])
        tag_id_to_center[tag.tag_id] = (cX, cY)
        center_to_tag_id[(cX, cY)] = tag.tag_id

        # 테두리 그리기
        corners = [(int(pt[0]), int(pt[1])) for pt in tag.corners]
        for i in range(4):
            cv2.line(frame, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)

    if len(tag_centers) >= 4 and printed_once:
        tag_center_list = list(center_to_tag_id.keys())  # [(x,y), ...]

        grid_corners = [
            find_closest(top_left, tag_center_list),
            find_closest(top_right, tag_center_list),
            find_closest(bottom_left, tag_center_list),
            find_closest(bottom_right, tag_center_list)
        ]
        printed_once = False
        print("📌 보정된 꼭짓점 위치(grid_corners):", grid_corners)


    if grid_corners:
        grid_points = generate_grid_points(grid_corners, horizontal_divisions, vertical_divisions)
        visualize_grid(frame, grid_points, ROWS, COLS)

    cv2.imshow("AprilTag Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("스크립트를 종료합니다.")
