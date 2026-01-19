import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 构建 MoveIt 配置
    # 注意：这里我们不再加载 custom joint_limits，而是使用默认配置
    # 这意味着机器人拥有完整的物理运动范围
    moveit_config = MoveItConfigsBuilder(
        "fairino16_v6", 
        package_name="fairino16_v6_moveit2_config"
    ).to_moveit_configs()

    # 定义测试节点
    benchmark_node = Node(
        package="algorithm_benchmark_pkg_04",
        executable="benchmark_node_04",
        name="benchmark_node_04",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits, # 默认限制
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        benchmark_node
    ])