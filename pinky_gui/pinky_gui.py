import os
import sys
import time
from pinky_lcd import LCD
from PIL import Image, ImageDraw, ImageFont

# 이미지 경로와 폰트 경로 설정
# images 폴더는 스크립트와 같은 디렉토리에 있어야 합니다.
IMAGE_DIR = "images"
FONT_PATH = "MaruBuri-Bold.ttf"  # 핑키 로봇에 복사한 폰트 파일명

# 품목 ID와 이미지 파일명을 연결하는 딕셔너리
item_images = {
    "1": "chrome-hearts.png",
    "2": "sunglasses.png",
    "3": "MacBook.png",
    "4": "iphone16pro.png",
    "5": "eraser.png",
    "6": "pen.png",
}

def display_on_lcd(item_id, item_amount):
    """
    입력받은 품목 ID와 수량을 Pinky LCD에 출력하는 함수
    """
    try:
        # Pinky LCD 객체 생성
        lcd = LCD()

        # 폰트 로딩. 폰트 파일이 없으면 기본 폰트 사용
        try:
            font = ImageFont.truetype(FONT_PATH, 25)
        except IOError:
            print(f"경고: 폰트 파일 '{FONT_PATH}'을(를) 찾을 수 없습니다. 기본 폰트를 사용합니다.")
            font = ImageFont.load_default()

        # 품목 이미지 파일명 가져오기
        image_file = item_images.get(item_id)
        if not image_file:
            print(f"경고: 품목 ID '{item_id}'에 해당하는 이미지를 찾을 수 없습니다.")
            lcd.close()
            return

        # 이미지 파일 경로 설정 및 존재 여부 확인
        image_path = os.path.join(IMAGE_DIR, image_file)
        if not os.path.exists(image_path):
            print(f"경고: 이미지 파일 '{image_path}'을(를) 찾을 수 없습니다.")
            lcd.close()
            return

        # LCD 화면 크기 설정 (320x240)
        img_width, img_height = 320, 240
        
        # 검은색 배경 이미지 생성
        img = Image.new('RGB', (img_width, img_height), color=(0, 0, 0))
        draw = ImageDraw.Draw(img)

        # 품목 이미지 불러오기 및 크기 조절
        item_img = Image.open(image_path)
        item_img = item_img.resize((150, 150))
        
        # 이미지를 화면 좌측에 배치
        img_x = 10
        img_y = (img_height - item_img.height) // 2
        img.paste(item_img, (img_x, img_y))

        # 텍스트 내용
        text1 = f"Item ID: {item_id}"
        text2 = f"Amount: {item_amount}"

        # 텍스트 위치 계산 및 그리기
        # 텍스트의 크기를 측정하여 중앙에 정렬
        text_bbox1 = draw.textbbox((0, 0), text1, font=font)
        text_bbox2 = draw.textbbox((0, 0), text2, font=font)
        text_height = text_bbox1[3] - text_bbox1[1]

        text_x = 170
        text_y1 = (img_height - (text_height * 2 + 20)) // 2
        text_y2 = text_y1 + text_height + 20

        draw.text((text_x, text_y1), text1, fill=(255, 255, 255), font=font)
        draw.text((text_x, text_y2), text2, fill=(255, 255, 255), font=font)
        
        # LCD에 최종 이미지 표시
        lcd.img_show(img)

        print(f"LCD에 'ID: {item_id}, 수량: {item_amount}'을(를) 출력했습니다.")
        
        # 화면을 잠시 유지 (옵션)
        time.sleep(10) # 10초 유지 후 종료
        
    except Exception as e:
        print(f"LCD 출력 중 오류 발생: {e}")
    finally:
        # 프로그램 종료 시 LCD 연결 종료
        if 'lcd' in locals() and lcd:
            lcd.close()

if __name__ == "__main__":
    # 터미널 인자 확인
    if len(sys.argv) != 3:
        print("사용법: python3 your_script_name.py <item_id> <item_amount>")
        sys.exit(1)
    
    # 인자로 받은 값으로 LCD에 출력
    item_id = sys.argv[1]
    item_amount = sys.argv[2]
    display_on_lcd(item_id, item_amount)
