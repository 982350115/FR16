from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("fairino16_v6_robot", package_name="fairino16_v6_moveit2_config").to_moveit_configs()

    # 2. 定义 Move Group 节点 (核心修改：添加 capabilities)
    # 这里的关键是添加 'move_group/ExecuteTaskSolutionCapability'
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "capabilities": "move_group/ExecuteTaskSolutionCapability"
            },
        ],
    )

    # 3. 定义 RViz 节点 (用于可视化)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 4. 定义 静态 TF 发布 (通常需要)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 5. 定义 Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 6. 定义 ros2_control (如果需要驱动真实机械臂或 Fake 控制器)
    # 注意：generate_demo_launch 通常会自动启动这个。如果你是在仿真/真机，可能需要保留。
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, str(moveit_config.package_path / "config/ros2_controllers.yaml")],
        output="screen",
    )

    # -----------------------------------------------------------------
    # 即使是仿真，这三个节点也是让机械臂动起来的“灵魂”
    # -----------------------------------------------------------------
    
    # 1. 关节状态发布器 (负责告诉 RViz 机械臂现在的姿态)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 2. 机械臂控制器 (负责接收 MoveIt 的规划并“移动”虚拟关节)
    # 请确保 "fairino16_v6_group_controller" 与 ros2_controllers.yaml 里的一致
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino16_controller", "--controller-manager", "/controller_manager"], # 原来是 fairino16_v6_group_controller
        output="screen",
    )

    # 3. 夹爪控制器 (负责开合虚拟夹爪)
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        rviz_node,
        move_group_node,
        ros2_control_node,          # 硬件管理器
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])