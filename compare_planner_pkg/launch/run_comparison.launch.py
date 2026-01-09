from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载 MoveIt 配置
    # 这里会自动读取 fairino16_v6_moveit2_config 包里的 URDF 和 SRDF
    moveit_config = MoveItConfigsBuilder("fairino16_v6_robot", package_name="fairino16_v6_moveit2_config").to_moveit_configs()

    # 2. 定义节点
    compare_node = Node(
        package="compare_planner_pkg",
        executable="compare_algorithms",
        name="compare_algorithms_node", # 名字必须和 C++ 代码里 Node 初始化的一致
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # 如果是仿真，这一项通常设为 True，但如果是 Mock 硬件或纯 Rviz，False 也没问题
            {"use_sim_time": True}, 
        ],
    )

    return LaunchDescription([
        compare_node,
    ])