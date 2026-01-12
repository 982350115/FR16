from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载 MoveIt 配置
    # 重点：pipelines 必须包含 ompl 和 pilz_industrial_motion_planner
    moveit_config = MoveItConfigsBuilder("fairino16_v6_robot", package_name="fairino16_v6_moveit2_config") \
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"]) \
        .to_moveit_configs()

    # 2. 实验节点
    benchmark_node = Node(
        package="planner_benchmark_pkg",
        executable="benchmark_node",
        name="benchmark_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines, # 这一行非常关键，传递算法配置
            # 确保加载 joint_limits，虽然 MoveItConfigsBuilder 默认会做，但显式传递更安全
            moveit_config.joint_limits, 
            {"use_sim_time": True}, 
        ],
    )

    return LaunchDescription([
        benchmark_node,
    ])