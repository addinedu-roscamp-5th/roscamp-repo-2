# 🤖 로봇 네비게이션 시스템

상태머신 기반 로봇 컨트롤러와 웨이포인트 팔로잉 시스템

## 📁 파일 구조

```
scripts/
├── grid_camera_app.py          # 메인 카메라 앱 (웨이포인트 설정 및 모니터링)
├── robot_controller.py         # 개별 로봇 컨트롤러 (상태머신 기반)
├── start_robot_controllers.py  # 멀티 로봇 런처
└── README_robot_control.md     # 이 문서
```

## 🎯 시스템 개요

1. **Grid Camera App**: 웨이포인트 설정, 로봇 추적, ROS2 토픽 발행
2. **Robot Controller**: PID 기반 네비게이션, 상태머신 제어
3. **Domain Bridge**: 도메인 간 토픽 브릿징 (config/domain_bridge_config.yaml)

## 📡 ROS2 토픽 구조

### 입력 토픽 (Robot Controller 구독)
- `/robot{i}/camera_pose` - 현재 로봇 위치 (PoseStamped)
- `/robot{i}/target_pose` - 목표 위치/웨이포인트 (PoseStamped)
- `/robot{i}/battery` - 배터리 레벨 (Float32)

### 출력 토픽 (Robot Controller 발행)
- `/robot{i}/cmd_vel` - 속도 제어 명령 (Twist)

## 🔄 로봇 상태머신

```
     IDLE ←→ NAVIGATING ←→ REACHED_GOAL
       ↓         ↓              ↑
     ERROR ←→ STOPPED ←----------┘
```

### 상태 설명
- **IDLE**: 대기 상태, 새로운 목표 대기
- **NAVIGATING**: 목표로 이동 중, PID 제어 활성
- **REACHED_GOAL**: 목표 도달, 잠시 정지 후 IDLE로 전환
- **ERROR**: 배터리 부족 등 에러 상태
- **STOPPED**: 수동 정지 상태

## 🚀 사용법

### 1. 단일 로봇 실행
```bash
# 로봇 1 컨트롤러 실행
python3 robot_controller.py 1

# 로봇 2 컨트롤러 실행  
python3 robot_controller.py 2
```

### 2. 멀티 로봇 실행
```bash
# 모든 로봇 컨트롤러 동시 실행
python3 start_robot_controllers.py
```

### 3. 웨이포인트 설정 및 미션 시작
1. `grid_camera_app.py` 실행
2. 카메라 그리드 4개 코너 설정
3. AprilTag 감지 활성화
4. "테스트 웨이포인트 설정" 클릭
5. "로봇 자동 제어 활성화" 체크
6. "웨이포인트 미션 시작" 클릭

## ⚙️ PID 파라미터 튜닝

`robot_controller.py`에서 다음 값들을 조정:

```python
# 거리 제어 PID
self.distance_pid = PIDController(
    kp=1.0,    # 비례 게인 (높을수록 빠른 반응)
    ki=0.0,    # 적분 게인 (정상상태 오차 제거)  
    kd=0.1     # 미분 게인 (오버슈트 억제)
)

# 각도 제어 PID  
self.angle_pid = PIDController(
    kp=2.0,    # 비례 게인
    ki=0.0,    # 적분 게인
    kd=0.2     # 미분 게인
)
```

## 🔧 주요 파라미터

```python
# 네비게이션 파라미터
goal_tolerance = 0.1        # 목표 도달 허용 오차 (m)
max_linear_speed = 0.5      # 최대 선속도 (m/s)
max_angular_speed = 1.0     # 최대 각속도 (rad/s)
waypoint_tolerance = 0.1    # 웨이포인트 도달 허용 오차 (m)
```

## 🐛 디버깅

### 로그 확인
- Robot Controller는 1Hz로 상태 정보 출력
- Grid Camera App에서 웨이포인트 진행 상황 확인

### 일반적인 문제
1. **로봇이 움직이지 않음**: ROS2 토픽 연결 상태 확인
2. **진동하는 움직임**: PID 게인값 조정 필요
3. **목표에 도달하지 못함**: goal_tolerance 값 증가

### ROS2 토픽 모니터링
```bash
# 현재 위치 확인
ros2 topic echo /robot1/camera_pose

# 목표 위치 확인  
ros2 topic echo /robot1/target_pose

# 속도 명령 확인
ros2 topic echo /robot1/cmd_vel
```

## 🔄 확장 가능성

1. **더 많은 로봇 추가**: `ROBOT_CONFIG`와 `ROBOT_IDS` 수정
2. **고급 경로 계획**: A* 경로를 실제 제어에 반영
3. **장애물 회피**: 실시간 장애물 감지 및 회피
4. **협업 작업**: 로봇 간 통신 및 작업 분배

## 📊 성능 최적화

- 제어 주기: 20Hz (0.05초)
- 로그 출력: 1Hz
- PID 제어는 실시간 오차 기반
- 각도 오차가 큰 경우 선속도 자동 감소

## 🚨 안전 기능

- 배터리 부족 시 자동 정지 (20% 미만)
- Ctrl+C로 모든 로봇 즉시 정지
- 목표 도달 시 자동 정지
- 최대 속도 제한 적용