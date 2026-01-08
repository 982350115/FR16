#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 创建节点
  auto const node = std::make_shared<rclcpp::Node>(
    "multi_point_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 1. 初始化 MoveGroupInterface (只声明一次！)
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "fairino16_v6_group");

  // ==================== 【关键修复】 ====================
  // 强制指定使用 OMPL 管线，防止系统自动切到 CHOMP
  move_group.setPlanningPipelineId("ompl"); 
  
  // 指定具体的算法 (RRT*)
  move_group.setPlannerId("RRTstarkConfigDefault");
  // ====================================================

  // 2. 配置规划参数
  move_group.setPlanningTime(15.0); 
  move_group.setNumPlanningAttempts(10);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.1);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 3. 定义目标点
  std::vector<geometry_msgs::msg::Pose> target_poses;

  // 获取当前位姿作为基准，避免 IK 无解
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

  // 目标点 1
  geometry_msgs::msg::Pose p1 = current_pose; 
  p1.position.x = 0.3; 
  p1.position.y = 0.1; 
  p1.position.z = 0.5;
  target_poses.push_back(p1);

  // 目标点 2
  geometry_msgs::msg::Pose p2 = current_pose; 
  p2.position.x = 0.3; 
  p2.position.y = -0.1; 
  p2.position.z = 0.5;
  target_poses.push_back(p2);

  // 4. 循环执行
  for (size_t i = 0; i < target_poses.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "---------------------------------------");
    RCLCPP_INFO(node->get_logger(), "正在前往第 %zu 个目标点...", i + 1);

    move_group.setPoseTarget(target_poses[i]);

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(node->get_logger(), "✅ 规划成功 (耗时: %.2fs)，开始执行...", my_plan.planning_time_);
      move_group.execute(my_plan);
      RCLCPP_INFO(node->get_logger(), "到达目标点 %zu ！", i + 1);
    } else {
      RCLCPP_ERROR(node->get_logger(), "❌ 目标点 %zu 规划失败！", i + 1);
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::shutdown();
  return 0;
}