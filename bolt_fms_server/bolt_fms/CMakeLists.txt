cmake_minimum_required(VERSION 3.8)
project(bolt_fms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencies ─────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2 REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/InboundUpdate.msg"
"msg/TaskSimple.msg"
DEPENDENCIES geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

# ─── Include Directories ───────────────────────────────────────
include_directories(include)

# ─── Executables (C++ Nodes) ───────────────────────────────────
add_executable(agent_manager_node src/agent_manager_node.cpp)
add_executable(task_manager_node src/task_manager_node.cpp)
add_executable(inventory_manager_node src/inventory_manager_node.cpp)
add_executable(traffic_manager_node src/traffic_manager_node.cpp)
add_executable(tcp_gateway src/tcp_gateway.cpp)
add_executable(bolt_fms_core src/bolt_fms_core.cpp)

target_link_libraries(tcp_gateway nlohmann_json::nlohmann_json)


# ─── Dependencies per target ───────────────────────────────────
ament_target_dependencies(agent_manager_node rclcpp std_msgs geometry_msgs)
ament_target_dependencies(task_manager_node rclcpp std_msgs)
ament_target_dependencies(inventory_manager_node rclcpp std_msgs)
ament_target_dependencies(traffic_manager_node rclcpp std_msgs)
ament_target_dependencies(tcp_gateway rclcpp std_msgs geometry_msgs sensor_msgs nlohmann_json)
ament_target_dependencies(bolt_fms_core rclcpp std_msgs geometry_msgs sensor_msgs nlohmann_json tf2)


# ─── Install Targets ───────────────────────────────────────────
install(TARGETS
  agent_manager_node
  task_manager_node
  inventory_manager_node
  traffic_manager_node
  bolt_fms_core
  tcp_gateway
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 리소스 설치
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

# python 스크립트를 실행 파일로 설치
install(PROGRAMS
  scripts/GUI/inbound_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# ❗ msg 디렉토리는 현재 사용 안하므로 제거 또는 주석 처리
# install(DIRECTORY msg DESTINATION share/${PROJECT_NAME}/msg)



# ─── Python Scripts ────────────────────────────────────────────
# ⚠ bolt_fms/bolt_fms 구조는 피하세요. scripts/ 디렉토리를 따로 사용 권장
install(PROGRAMS
  # scripts/qtmonitor_tag_tracker.py
  scripts/grid_camera_app.py
  scripts/integrated_grid_camera_app.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
