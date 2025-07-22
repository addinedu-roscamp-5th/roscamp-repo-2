import cv2
import numpy as np

# 이미지 불러오기
image = cv2.imread('images/capture_1753157742_0.jpg')
if image is None:
    print("이미지를 불러올 수 없습니다.")
    exit()

clone = image.copy()
points = []
draw_grid = False

# 🟡 가로/세로 격자 분할 수 (자유롭게 변경 가능)
rows = 12  # 가로줄 (위-아래 방향)
cols = 24    # 세로줄 (좌-우 방향)

def interpolate(p1, p2, t):
    """ 두 점 p1, p2 사이의 t(0~1) 비율 위치 계산 """
    return (int(p1[0] + t * (p2[0] - p1[0])), int(p1[1] + t * (p2[1] - p1[1])))

def draw_trapezoid_grid(img, pts, h_div, v_div):
    p0, p1, p2, p3 = pts

    # 🟡 가로줄: 왼쪽(p0-p2), 오른쪽(p1-p3)
    lefts = [interpolate(p0, p2, i / h_div) for i in range(h_div + 1)]
    rights = [interpolate(p1, p3, i / h_div) for i in range(h_div + 1)]

    for l, r in zip(lefts, rights):
        cv2.line(img, l, r, (255, 255, 0), 1)

    # 🟡 세로줄: 위(p0-p1), 아래(p2-p3)
    tops = [interpolate(p0, p1, i / v_div) for i in range(v_div + 1)]
    bottoms = [interpolate(p2, p3, i / v_div) for i in range(v_div + 1)]

    for t, b in zip(tops, bottoms):
        cv2.line(img, t, b, (0, 255, 255), 1)

# 마우스 콜백 함수
def click_event(event, x, y, flags, param):
    global points, image, draw_grid

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(image, f"{len(points)}", (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if len(points) == 4:
            cv2.line(image, points[0], points[1], (255, 0, 0), 2)
            cv2.line(image, points[1], points[3], (255, 0, 0), 2)
            cv2.line(image, points[3], points[2], (255, 0, 0), 2)
            cv2.line(image, points[2], points[0], (255, 0, 0), 2)

            draw_grid = True

cv2.namedWindow("Select 4 Points")
cv2.setMouseCallback("Select 4 Points", click_event)

while True:
    display = image.copy()

    if draw_grid and len(points) == 4:
        draw_trapezoid_grid(display, points, rows, cols)

    cv2.imshow("Select 4 Points", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('r'):
        image = clone.copy()
        points = []
        draw_grid = False
    elif key == ord('q'):
        break

cv2.destroyAllWindows()
