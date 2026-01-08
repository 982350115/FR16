from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 加载配置 (确保包名和你之前的一致)
    moveit_config = MoveItConfigsBuilder("fairino16_v6_robot", package_name="fairino16_v6_moveit2_config").to_moveit_configs()

    # 定义使用默认算法的节点
    default_planner_node = Node(
        package="compare_planner_pkg",
        executable="default_planner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False}, 
        ],
    )

    return LaunchDescription([
        default_planner_node,
    ])