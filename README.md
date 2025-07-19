# roscamp-repo-2

ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 2팀 저장소.

# Installation

```bash
cd ~/colcon_ws/src
git clone https://github.com/addinedu-roscamp-5th/roscamp-repo-2.git --resursive
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y # or 'rd'
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install # or 'cb'
source ./install/local_setup.bash # or 'sb'
```

# Bolt_fms

# common_pkgs

공통으로 사용하는 패키지

- apriltag_ros

# Jetcobot

Jetcobot에서 사용되는 ROS 패키지

- jetcobot_bringup
- jetcobot_description
- jetcobot_moveit_config
- jetcobot_moveit_picker