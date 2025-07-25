# roscamp-repo-2

ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 2팀 저장소.

# Installation

```bash
cd ~/colcon_ws/src
git clone https://github.com/addinedu-roscamp-5th/roscamp-repo-2.git --recursive
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y # or 'rd'
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install # or 'cb'
source ./install/local_setup.bash # or 'sb'
```

# Bolt_fms
## How to build
> 워크스페이스로 이동 
```bash
cd ~/colcon_ws
```
> bolt_fms 패키지 빌드 후 소싱
> --symlink-install 옵션이 Python 패키지에서는 .py 파일들을 소스 디렉토리에서 직접 참조할 수 있도록 심볼릭 링크를 생성하는 방식으로 동작하는데 cpp 패키지가 베이스라 --symlink-install 빼주셔야 합니다
```bash
colcon build --packages-select bolt_fms
source ./install/local_setup.bash # or 'sb'
```
> 파이썬 외부라이브러리용 가상환경 생성과 소싱 및 종속성 설치
```bash
virtualenv -p python3 ./venv # Make a virtual env and activate it
source ./venv/bin/activate
touch ./venv/COLCON_IGNORE # Make sure that colcon does not try to build the venv
pip install -r src/roscamp-repo-2/requirements.txt
```

# common_pkgs

공통으로 사용하는 패키지

- apriltag_ros

# Jetcobot

Jetcobot에서 사용되는 ROS 패키지

- jetcobot_bringup
- jetcobot_description
- jetcobot_moveit_config
- jetcobot_moveit_picker
