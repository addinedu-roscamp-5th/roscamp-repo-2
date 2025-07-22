import cv2
import time

cap = cv2.VideoCapture(3)
if not cap.isOpened():
    print("오류: 카메라를 열 수 없습니다.")
    exit()

save_count = 0
print("카메라 피드 시작 — 'c'를 누르면 이미지 저장, 'q'를 누르면 종료")

while True:
    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임을 읽을 수 없습니다.")
        break

    cv2.imshow("Camera Feed", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        # q 누르면 종료
        print("종료 키(q) 입력 — 프로그램 종료")
        break
    elif key == ord('c'):
        # c 누르면 현재 프레임 저장
        filename = f"images/capture_{int(time.time())}_{save_count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"이미지 저장됨: {filename}")
        save_count += 1

cap.release()
cv2.destroyAllWindows()
