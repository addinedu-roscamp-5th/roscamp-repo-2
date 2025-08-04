import cv2
import numpy as np
import glob
import sys

# 1) 체스판 패턴 설정
PATTERN_SIZE = (10, 7)
SQUARE_SIZE  = 0.020

# 3D 좌표 준비
objp = np.zeros((PATTERN_SIZE[1]*PATTERN_SIZE[0], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []
image_size = None

# 2) 패턴 이미지 불러오기
images = glob.glob("images/*.jpg")
print(f"캘리브레이션용 이미지 개수: {len(images)}")
if len(images) == 0:
    print("오류: images 폴더에 .jpg 파일이 없습니다.")
    sys.exit(1)

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"[경고] 이미지를 읽을 수 없음: {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if image_size is None:
        image_size = gray.shape[::-1]   # (width, height)

    # 체스판 코너 검출
    ret, corners = cv2.findChessboardCorners(
        gray, PATTERN_SIZE,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not ret:
        print(f"[경고] 코너 검출 실패: {fname}")
        continue

    # 코너 정밀화
    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

    objpoints.append(objp.copy())
    imgpoints.append(corners2)

    # 디버그: 코너 그리기
    cv2.drawChessboardCorners(img, PATTERN_SIZE, corners2, ret)
    cv2.imshow('Chessboard', img)
    cv2.waitKey(200)

cv2.destroyAllWindows()

# 최소한 한 쌍 이상은 잡혔는지 확인
if len(objpoints) < 1:
    print("오류: 유효한 체스판 이미지가 충분히 없습니다.")
    sys.exit(1)

# 3) 보정 수행
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, image_size, None, None
)

print("캘리브레이션 RMS error:", ret)
print("cameraMatrix:\n", cameraMatrix)
print("distCoeffs:\n", distCoeffs.ravel())

# 4) 결과 저장
np.save("cameraMatrix.npy", cameraMatrix)
np.save("distCoeffs.npy", distCoeffs)
print("✅ cameraMatrix.npy, distCoeffs.npy 저장 완료")
