cmake_minimum_required(VERSION 3.8)
project(algorithm_benchmark_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED) # <--- 必须有这个
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(benchmark_node src/benchmark_node.cpp)

ament_target_dependencies(benchmark_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core # <--- 必须链接这个
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS benchmark_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件 (保持不变)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()