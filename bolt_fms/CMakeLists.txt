cmake_minimum_required(VERSION 3.8)
project(bolt_fms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

install(PROGRAMS
  scripts/auto_grid_and_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

