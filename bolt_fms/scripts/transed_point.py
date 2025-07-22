import numpy as np
import cv2

# 4개의 실제 이미지상 꼭짓점 좌표 (float32)
grid_corners = np.array([
    [0, 0],  # top-left
    [100, 10],  # top-right
    [10, 50],  # bottom-left
    [120, 20],  # bottom-right
], dtype=np.float32)

# 상대 좌표계를 정의한 정사각형 기준 좌표
unit_square = np.array([
    [0.0, 0.0],
    [1.0, 0.0],
    [0.0, 1.0],
    [1.0, 1.0],
], dtype=np.float32)

# 변환 행렬 계산
M = cv2.getPerspectiveTransform(grid_corners, unit_square)

# 예시: 상대 좌표 (0.5, 0.3) → 이미지 좌표로 변환
relative_point = np.array([[[84.0, 11.8]]], dtype=np.float32)
projected_point = cv2.perspectiveTransform(relative_point, M)

px, py = projected_point[0][0]
print(f"변환된 이미지 좌표: ({px:.1f}, {py:.1f})")

# 시각화
# cv2.circle(frame, (int(px), int(py)), 5, (0, 255, 255), -1)
