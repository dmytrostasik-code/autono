cmake_minimum_required(VERSION 3.8)
project(ardurover_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(path_recorder_node src/path_recorder_node.cpp)
target_include_directories(path_recorder_node PRIVATE include)
ament_target_dependencies(path_recorder_node rclcpp nav_msgs)

add_executable(trajectory_controller_node
  src/trajectory_controller_node.cpp
  src/ardurover_controller.cpp)
target_include_directories(trajectory_controller_node PRIVATE include)
ament_target_dependencies(trajectory_controller_node rclcpp geometry_msgs nav_msgs mavros_msgs visualization_msgs)

add_executable(path_scorer_node src/path_scorer_node.cpp)
target_include_directories(path_scorer_node PRIVATE include)
ament_target_dependencies(path_scorer_node rclcpp nav_msgs)

install(TARGETS
  path_recorder_node
  trajectory_controller_node
  path_scorer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
