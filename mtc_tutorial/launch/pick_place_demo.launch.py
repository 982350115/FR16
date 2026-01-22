from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ===============================================================
    # 1. 构建 MoveIt 配置
    # ===============================================================
    moveit_config = MoveItConfigsBuilder("fairino16_v6_robot", package_name="fairino16_v6_moveit2_config").to_moveit_configs()

    # ===============================================================
    # 2. 定义各个节点
    # ===============================================================

    # [Move Group] 路径规划核心
    # 必须加载 capabilities: ExecuteTaskSolutionCapability
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

    # [RViz] 可视化
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

    # [TF] 静态坐标变换
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # [Robot State Publisher] 发布 TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # [ros2_control] 硬件管理器
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, str(moveit_config.package_path / "config/ros2_controllers.yaml")],
        output="screen",
    )

    # -------------------------------------------------------------
    # [控制器 Spawner] 必须与 ros2_controllers.yaml 中的名称完全一致
    # -------------------------------------------------------------

    # 1. 关节状态
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 2. 机械臂控制器 (注意：你的 YAML 里叫 fairino16_controller，不是 fairino16_v6_group_controller)
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino16_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 3. 夹爪控制器 (之前报错缺失的部分)
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # [Demo 程序] 你的 MTC 代码
    mtc_demo_node = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # ===============================================================
    # 3. 返回 LaunchDescription
    # ===============================================================
    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        rviz_node,
        ros2_control_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller, # 必须启动这个
        move_group_node,
        mtc_demo_node,
    ])