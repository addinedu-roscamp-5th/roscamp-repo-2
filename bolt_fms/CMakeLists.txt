cmake_minimum_required(VERSION 3.8)
project(bolt_fms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(PySide6 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# include 
include_directories(include)


add_executable(agent_manager_node src/agent_manager_node.cpp)
add_executable(task_manager_node src/task_manager_node.cpp)
add_executable(inventory_manager_node src/inventory_manager_node.cpp)
add_executable(traffic_manager_node src/traffic_manager_node.cpp)
add_executable(monitoring_manager_node src/monitoring_manager_node.cpp)

ament_target_dependencies(agent_manager_node rclcpp std_msgs geometry_msgs)
ament_target_dependencies(task_manager_node rclcpp std_msgs)
ament_target_dependencies(inventory_manager_node rclcpp std_msgs)
ament_target_dependencies(traffic_manager_node rclcpp std_msgs)
ament_target_dependencies(monitoring_manager_node rclcpp std_msgs geometry_msgs)
# ament_target_dependencies(my_node_target_name rclcpp std_msgs ${PROJECT_NAME})

install(TARGETS
  agent_manager_node
  task_manager_node
  inventory_manager_node
  traffic_manager_node
  monitoring_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/py_node.py
  bolt_fms/tag_tracker_node.py
  bolt_fms/qtmonitor_tag_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

