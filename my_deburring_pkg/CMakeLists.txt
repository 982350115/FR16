cmake_minimum_required(VERSION 3.8)
project(my_deburring_pkg)

# 1. 强制使用 C++17 标准 (MoveIt 2 需要)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
# 这是 MoveIt 的 C++ 接口核心库
find_package(moveit_ros_planning_interface REQUIRED)

# 3. 定义可执行文件
# 确保你的源代码文件名是 src/multi_point_planner.cpp
add_executable(multi_point_planner src/multi_point_planner.cpp)

# 4. 链接依赖
# ament_target_dependencies 会自动处理 include 路径和库链接
ament_target_dependencies(multi_point_planner
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

# 5. 安装规则
# 这样你才能使用 `ros2 run my_deburring_pkg multi_point_planner` 运行它
install(TARGETS multi_point_planner
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()