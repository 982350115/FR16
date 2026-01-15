import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. 获取本包路径，用于定位自定义的 limits 文件
    pkg_share = get_package_share_directory('algorithm_benchmark_pkg_03')
    constrained_limits_path = os.path.join(pkg_share, 'config', 'joint_limits_constrained.yaml')

    # 2. 构建 MoveIt 配置
    # 技巧：使用 .joint_limits() 方法加载我们的自定义文件
    # 注意：package_name 必须指向原始的配置包
    moveit_config = MoveItConfigsBuilder(
        "fairino16_v6", 
        package_name="fairino16_v6_moveit2_config"
    ).joint_limits(
        file_path=constrained_limits_path
    ).to_moveit_configs()

    # 3. 定义测试节点
    benchmark_node = Node(
        package="algorithm_benchmark_pkg_03",
        executable="benchmark_node_03",
        name="benchmark_node_03",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits, # 这里加载的就是我们修改过的限制
            moveit_config.planning_pipelines,
            # 如果需要加载 pilz，确保 planning_pipelines 包含了它
            # 或者显式加载 pilz_industrial_motion_planner
        ],
    )

    return LaunchDescription([
        benchmark_node
    ])