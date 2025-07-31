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
        *   qmonitor_robot_stat도e_machine.py 창에서 게인값을 조절하고, yaml을 저장하면 config로 사용하게 됩니다.
        
5.  **향후 개발**
    *   관제에서 qmonitor는 통합하실예정이라고 합니다. target_pose를 받고 이동하는 기능 고도화 이후, ros node화 하면 될것으로 예상됩니다.
