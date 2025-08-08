#!/usr/bin/env python3
"""
멀티 로봇 컨트롤러 런처
여러 로봇 컨트롤러를 동시에 실행하는 스크립트
"""

import subprocess
import sys
import signal
import time
from pathlib import Path


# 로봇 설정 (robot_id 리스트)
ROBOT_IDS = [1, 2]  # 확장 가능


class MultiRobotLauncher:
    def __init__(self):
        self.processes = []
        self.running = True
        
        # Ctrl+C 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Ctrl+C 시그널 핸들러"""
        _ = sig, frame  # 사용하지 않는 파라미터 표시
        print(f"\n🛑 받은 종료 신호. 모든 로봇 컨트롤러를 종료합니다...")
        self.running = False
        self.stop_all_controllers()
        sys.exit(0)
    
    def start_controller(self, robot_id):
        """개별 로봇 컨트롤러 시작"""
        script_path = Path(__file__).parent / "robot_controller.py"
        
        try:
            process = subprocess.Popen([
                sys.executable, str(script_path), str(robot_id)
            ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            
            self.processes.append({
                'robot_id': robot_id,
                'process': process
            })
            
            print(f"✅ Robot {robot_id} Controller started (PID: {process.pid})")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start Robot {robot_id} Controller: {e}")
            return False
    
    def stop_all_controllers(self):
        """모든 로봇 컨트롤러 정지"""
        for proc_info in self.processes:
            robot_id = proc_info['robot_id']
            process = proc_info['process']
            
            try:
                if process.poll() is None:  # 프로세스가 아직 실행 중
                    print(f"🔄 Stopping Robot {robot_id} Controller...")
                    process.terminate()
                    
                    # 5초 대기 후 강제 종료
                    try:
                        process.wait(timeout=5)
                        print(f"✅ Robot {robot_id} Controller stopped gracefully")
                    except subprocess.TimeoutExpired:
                        print(f"⚠️ Force killing Robot {robot_id} Controller...")
                        process.kill()
                        process.wait()
                        print(f"💀 Robot {robot_id} Controller force killed")
                else:
                    print(f"ℹ️ Robot {robot_id} Controller was already stopped")
                    
            except Exception as e:
                print(f"❌ Error stopping Robot {robot_id} Controller: {e}")
        
        self.processes.clear()
    
    def check_processes(self):
        """프로세스 상태 확인 및 재시작"""
        for proc_info in self.processes[:]:  # 복사본으로 반복
            robot_id = proc_info['robot_id']
            process = proc_info['process']
            
            if process.poll() is not None:  # 프로세스가 종료됨
                print(f"⚠️ Robot {robot_id} Controller crashed. Restarting...")
                self.processes.remove(proc_info)
                time.sleep(1)  # 잠시 대기
                self.start_controller(robot_id)
    
    def run(self):
        """메인 실행 루프"""
        print("🚀 Multi-Robot Controller Launcher Starting...")
        print(f"📋 Robot IDs: {ROBOT_IDS}")
        
        # 모든 로봇 컨트롤러 시작
        success_count = 0
        for robot_id in ROBOT_IDS:
            if self.start_controller(robot_id):
                success_count += 1
            time.sleep(0.5)  # 순차 시작을 위한 짧은 대기
        
        if success_count == 0:
            print("❌ No controllers started successfully. Exiting...")
            return
        
        print(f"✅ {success_count}/{len(ROBOT_IDS)} controllers started successfully")
        print("🔄 Monitoring processes... (Press Ctrl+C to stop all)")
        
        # 모니터링 루프
        try:
            while self.running:
                time.sleep(2.0)  # 2초마다 체크
                self.check_processes()
                
        except KeyboardInterrupt:
            pass  # 시그널 핸들러에서 처리됨
        
        print("👋 Multi-Robot Controller Launcher finished")


def main():
    """메인 함수"""
    launcher = MultiRobotLauncher()
    launcher.run()


if __name__ == '__main__':
    main()