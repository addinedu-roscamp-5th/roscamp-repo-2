# ROS Camp Repository 2 - 자율주행 물류 로봇 시스템

ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 2팀 프로젝트입니다.  
본 프로젝트는 **Fleet Management System(FMS)**을 중심으로 한 통합 물류 자동화 시스템을 구현합니다.

## 🏗️ 시스템 아키텍처

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Bolt Mall     │    │    Bolt FMS      │    │   Robot Fleet   │
│   Web System    │◄──►│   Core Server    │◄──►│   Management    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Database &     │    │   AprilTag       │    │   Multi-Robot   │
│  Inventory      │    │   Navigation     │    │   Coordination  │
│  Management     │    │   System         │    │   & Control     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 📦 주요 구성 요소

### 🌐 Bolt Mall Web System
완전한 전자상거래 플랫폼으로 물류 자동화와 연동됩니다.

**주요 기능:**
- **상품 관리**: 상품 정보, 카테고리, 가격 관리
- **주문 처리**: 실시간 주문 접수 및 처리
- **재고 관리**: 자동 재고 추적 및 업데이트
- **물류 연동**: FMS와 실시간 연동을 통한 자동 출고

**기술 스택:**
- **Backend**: FastAPI (Python)
- **Database**: SQLAlchemy ORM, MySQL
- **Frontend**: HTML/CSS/JavaScript
- **API**: RESTful API 설계

### 🤖 Bolt FMS (Fleet Management System)
다중 로봇 관제 및 작업 할당을 담당하는 핵심 시스템입니다.

**핵심 기능:**
- **TCP 서버**: 다중 클라이언트 연결 및 실시간 통신
- **로봇 상태 관리**: 배터리, 위치, 속도 모니터링
- **작업 스케줄링**: 효율적인 작업 분배 알고리즘
- **AprilTag 기반 위치추적**: 고정밀 실내 위치 시스템

**구성 요소:**
```cpp
// 핵심 노드들
- bolt_fms_core: TCP 게이트웨이 및 상태 관리
- agent_manager_node: 로봇 에이전트 관리
- task_manager_node: 작업 할당 및 스케줄링
- traffic_manager_node: 경로 계획 및 충돌 방지
- inventory_manager_node: 재고 위치 관리
```

### 🔄 Robot State Machine System
정교한 상태 기반 로봇 제어 시스템입니다.

**상태 흐름:**
```
IDLE → ROTATE_TO_GOAL → MOVE_TO_GOAL → ROTATE_TO_FINAL → GOAL_REACHED
  ▲                                                              │
  └──────────────── TASK_COMPLETE ──────────────────────────────┘
```

**PID 제어 시스템:**
- **선속도 제어**: 목표점까지의 거리 기반
- **각속도 제어**: 목표 각도와의 오차 기반
- **동적 파라미터 조정**: YAML 설정을 통한 실시간 튜닝

### 🚧 Obstacle Detection & Avoidance
YOLO 기반 실시간 장애물 감지 및 회피 시스템입니다.

**구성 요소:**
- **YOLOv8 감지**: 실시간 객체 감지
- **Safety Bubble**: LiDAR 기반 안전 거리 유지
- **Domain Bridge**: 로봇 간 통신 네트워크

### 🦾 Jetcobot Arm Control
6축 매니퓰레이터 제어 및 작업 수행 시스템입니다.

**기능:**
- **MoveIt2 통합**: 고급 경로 계획
- **Task 기반 제어**: LOAD/UNLOAD 작업 자동화
- **충돌 감지**: 실시간 충돌 방지
- **Pick & Place**: 정밀한 물체 조작

### 📱 Pinky GUI System
사용자 친화적인 LCD 디스플레이 시스템입니다.

**특징:**
- **실시간 상태 표시**: 품목 정보 및 수량 표시
- **시각적 피드백**: 이미지와 텍스트 조합
- **한글 폰트 지원**: 직관적인 사용자 인터페이스

## 🚀 설치 및 실행

### 기본 환경 설정

```bash
# 1. 저장소 클론
cd ~/colcon_ws/src
git clone https://github.com/addinedu-roscamp-5th/roscamp-repo-2.git --recursive

# 2. 의존성 설치
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. 패키지 빌드
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ./install/local_setup.bash
```

### Python 가상환경 설정

```bash
# 가상환경 생성 및 활성화
virtualenv -p python3 ./venv
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE

# Python 의존성 설치
pip install -r src/roscamp-repo-2/requirements.txt
```

### Bolt FMS 서버 실행

```bash
# C++ 코어 서버 빌드 (--symlink-install 제외)
colcon build --packages-select bolt_fms
source ./install/local_setup.bash

# 핵심 노드 실행
ros2 launch bolt_fms fms_server_launch.py
```

### Bolt Mall 웹 서버 실행

```bash
cd bolt_mall_web

# 데이터베이스 초기화
python create_tables.py

# 웹 서버 시작
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 로봇 제어 시스템 실행

```bash
# 상태 머신 노드
ros2 run boltbot_state_machine robot_state_machine

# 장애물 감지 시스템
ros2 launch boltbot_obstacle obstacle_yolo_launch.py

# 작업 수신 노드
ros2 run boltbot_task_receiver task_receiver_node
```

### 모니터링 GUI 실행

```bash
# 로봇 상태 모니터링
ros2 run boltbot qmonitor_robot_state_machine

# Jetcobot 제어 GUI
ros2 run jetcobot_moveit_picker picker_gui
```

## 🔧 시스템 설정

### PID 파라미터 튜닝

```yaml
# pid_config.yaml
linear_pid:
  kp: 1.0
  ki: 0.0
  kd: 0.1
  max_output: 0.3
  
angular_pid:
  kp: 2.0
  ki: 0.0
  kd: 0.2
  max_output: 1.0
```

### 도메인 브리지 설정

```yaml
# bridge.yaml
topics:
  - topic: /robot_1/cmd_vel
    type: geometry_msgs/msg/Twist
    qos:
      reliability: reliable
```

## 📊 데이터베이스 스키마

### 주요 테이블 구조

```sql
-- 상품 정보
CREATE TABLE Item (
    item_id INT PRIMARY KEY,
    item_name VARCHAR(20),
    category VARCHAR(20),
    price DECIMAL(10,2),
    dimension_x/y/z DECIMAL(10,2)
);

-- 재고 관리
CREATE TABLE Inventory (
    inventory_id INT PRIMARY KEY,
    item_id INT,
    location_id INT,
    quantity INT,
    FOREIGN KEY (item_id) REFERENCES Item(item_id)
);

-- 주문 처리
CREATE TABLE Orders (
    order_id INT PRIMARY KEY,
    order_date DATETIME,
    total_amount DECIMAL(10,2),
    status VARCHAR(20)
);
```

## 🤝 시스템 통합 흐름

1. **주문 접수**: Bolt Mall에서 고객 주문 처리
2. **작업 생성**: FMS에서 자동으로 픽킹 작업 생성
3. **로봇 할당**: 최적의 로봇에게 작업 할당
4. **경로 계획**: AprilTag 기반 정밀 네비게이션
5. **장애물 회피**: YOLO + LiDAR 기반 안전 주행
6. **작업 수행**: Jetcobot을 통한 자동 픽킹
7. **상태 업데이트**: 실시간 작업 진행 상황 모니터링

## 🔬 기술적 특징

### 실시간 통신
- **TCP/IP**: 고성능 멀티 클라이언트 서버
- **ROS2 DDS**: 분산 시스템 간 안정적 통신
- **Domain Bridge**: 네트워크 세그멘테이션

### AI/ML 통합
- **YOLOv8**: 실시간 객체 감지
- **AprilTag**: 정밀 위치 추적
- **PID 제어**: 적응형 제어 알고리즘

### 확장성
- **모듈형 아키텍처**: 독립적 노드 구조
- **플러그인 시스템**: 새로운 로봇 타입 쉽게 추가
- **마이크로서비스**: 각 기능별 독립적 스케일링

## 👥 기여자

ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 2팀

*변정희, 김규철, 김진석, 박은영, 정채연, 조재혁*
