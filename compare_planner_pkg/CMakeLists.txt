cmake_minimum_required(VERSION 3.8)
project(compare_planner_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) # 用于显示 Marker
find_package(tf2 REQUIRED)                # 用于四元数转换
find_package(tf2_geometry_msgs REQUIRED)

# 2. 编译可执行文件
# 你的源代码文件名为 src/compare_algorithms.cpp
add_executable(compare_algorithms src/compare_algorithms.cpp)

# 3. 链接库文件
ament_target_dependencies(compare_algorithms
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# 4. 安装规则 (Install)
# 安装可执行文件
install(TARGETS
  compare_algorithms
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件夹 (非常重要！否则 ros2 launch 找不到文件)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()