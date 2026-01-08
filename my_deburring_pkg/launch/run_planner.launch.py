from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载 MoveIt 配置
    # 请确保 "fairino16_v6_moveit2_config" 是你正确的配置包名称
    moveit_config = MoveItConfigsBuilder("fairino16_v6_moveit2_config", package_name="fairino16_v6_moveit2_config").to_moveit_configs()

    # 2. 定义节点
    planner_node = Node(
        package="my_deburring_pkg",
        executable="multi_point_planner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # 有时候还需要这个参数来让 MoveIt 知道该用哪个规划管线
            {"use_sim_time": False}, 
        ],
    )

    return LaunchDescription([
        planner_node,
    ])