from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    
    # ==================== 核心修改区域 ====================
    # 方法：显式指定 package_name 参数
    # 第一个参数是机器人名字 (robot_name)，通常是 fairino16_v6
    # package_name 参数强制指定包所在的文件夹名
    moveit_config = MoveItConfigsBuilder(
        "fairino16_v6",  # 这里填机器人的名字（不带 _moveit_config）
        package_name="fairino16_v6_moveit2_config" # 这里填真实的包名
    ).to_moveit_configs()
    # ====================================================

    # ... 后面的 Node 定义代码保持不变 ...
    # 2. 定义测试节点
    benchmark_node = Node(
        package="algorithm_benchmark_pkg",
        executable="benchmark_node",
        name="benchmark_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # 如果 joint_limits.yaml 需要手动加载，MoveItConfigsBuilder 通常会自动处理
            # 但如果 PTP 还是报错，可能需要检查该配置包是否正确加载了 limits
        ],
    )

    return LaunchDescription([
        benchmark_node
    ])