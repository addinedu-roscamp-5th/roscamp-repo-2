import cv2
import numpy as np

# 원본 이미지
image = cv2.imread('images/capture_1753157742_0.jpg')
h_orig, w_orig = image.shape[:2]

# 변환할 원본 4점 (좌상, 우상, 좌하, 우하)
pts_src = np.float32([
    [75, 113],    # 좌상
    [584, 123],   # 우상
    [49, 361],    # 좌하
    [599, 372]    # 우하
])

# 변환 후 사각형 크기 결정
width = 600
height = 400
pts_dst = np.float32([
    [0, 0],
    [width - 1, 0],
    [0, height - 1],
    [width - 1, height - 1]
])

# 투시 변환 행렬
M = cv2.getPerspectiveTransform(pts_src, pts_dst)

# 투시 변환 결과 (사각형만)
warped = cv2.warpPerspective(image, M, (width, height))

# 변환된 결과를 다시 원래 위치에 붙이기 위한 역변환
M_inv = cv2.getPerspectiveTransform(pts_dst, pts_src)

# 원본과 같은 크기로 초기화
warped_back = cv2.warpPerspective(warped, M_inv, (w_orig, h_orig))

# 마스킹 처리: 변환된 영역만 원본에 덧붙이기
mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
cv2.fillConvexPoly(mask, pts_src.astype(int), 255)

# 이미지 합성: 마스크 영역만 warped_back 사용
result = image.copy()
result[mask == 255] = warped_back[mask == 255]

# 결과 보기
cv2.imshow("Result with embedded warped area", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
