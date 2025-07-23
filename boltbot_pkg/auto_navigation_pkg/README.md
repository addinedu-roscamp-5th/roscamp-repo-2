# 사용법

1.  **관제용 카메라 포즈 발행 기능 수행**
    *   관제를 위해 카메라 포즈를 발행하는 노드를 실행합니다.

2.  **로봇 상태 머신 실행**
    *   `python3 robot_state_machine.py`를 실행하여 로봇의 상태 머신을 시작합니다.

3.  **QMonitor 로봇 상태 머신 실행**
    *   `python3 qmonitor_robot_state_machine.py`를 실행하여 로봇 상태 머신을 모니터링합니다.

4.  **에러 및 PID 게인 값 조절**
    *   에러 데이터를 확인하고 PID 게인 값을 조절하려면 다음을 수행합니다.
        *   **Visualization Plot 띄우기**: 에러 데이터 2개를 플롯으로 시각화합니다.
        *   **Dynamic Reconfiguration**: `rqt_reconfigure`를 실행하고, `configuration > dynamic Reconfiguration`에서 `robotgoalcontroller`를 선택하여 PID 게인 값을 동적으로 조절합니다.
