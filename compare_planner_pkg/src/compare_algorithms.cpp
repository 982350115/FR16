#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

// 辅助工具：发布可视化标记
void publish_markers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, 
                    const std::vector<geometry_msgs::msg::Pose>& poses) 
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "target_points";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.08; marker.scale.y = 0.08; marker.scale.z = 0.08; 
    marker.color.a = 1.0; marker.lifetime = rclcpp::Duration::from_seconds(0);

    for (size_t i = 0; i < poses.size(); ++i) {
        marker.id = i;
        marker.pose = poses[i];
        marker.color.r = (i==0)? 1.0 : 0.0;
        marker.color.g = (i==1)? 1.0 : 0.0;
        marker.color.b = (i==2)? 1.0 : 0.0;
        publisher->publish(marker);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("compare_algorithms_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "fairino16_v6_group");

  // ==================== 1. 算法设置 ====================
  // std::string planner_id = "RRTConnectkConfigDefault"; 
  std::string planner_id = "RRTstarkConfigDefault"; 

  RCLCPP_INFO(node->get_logger(), "当前使用的算法: %s", planner_id.c_str());
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId(planner_id);
  move_group.setPlanningTime(5.0); 
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // ==================== 2. 定义目标点 ====================
  std::vector<geometry_msgs::msg::Pose> target_poses;
  tf2::Quaternion q;
  q.setRPY(0, 3.14159/2, 0); 

  // 点1 (红): 左
  geometry_msgs::msg::Pose p1;
  p1.orientation.x = q.x(); p1.orientation.y = q.y(); p1.orientation.z = q.z(); p1.orientation.w = q.w();
  p1.position.x = 0.0; p1.position.y = 0.5; p1.position.z = 0.4;
  target_poses.push_back(p1);

  // 点2 (绿): 右
  geometry_msgs::msg::Pose p2 = p1;
  p2.position.y = -0.5; 
  target_poses.push_back(p2);

  // 点3 (蓝): 中
  geometry_msgs::msg::Pose p3 = p1;
  p3.position.x = 0.4; p3.position.y = 0.0; p3.position.z = 0.6;
  target_poses.push_back(p3);

  rclcpp::sleep_for(std::chrono::seconds(2));
  publish_markers(marker_pub, target_poses);

  // ==================== 3. 循环执行 (强化版状态接力) ====================
  
  // 保存上一段轨迹的【最后一个点】
  trajectory_msgs::msg::JointTrajectoryPoint last_point;
  std::vector<std::string> last_joint_names;
  bool has_moved_once = false; 

  for (size_t i = 0; i < target_poses.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "---------------------------------------");
      RCLCPP_INFO(node->get_logger(), ">>> 准备前往目标点 %zu ...", i+1);
      
      move_group.setPoseTarget(target_poses[i]);
      
      // --- 核心修复：按名字强制同步起点 ---
      if (has_moved_once) {
          RCLCPP_INFO(node->get_logger(), "  [状态接力] 正在将起点设置为上一段的终点...");
          
          // 获取当前状态的副本
          moveit::core::RobotState start_state(*move_group.getCurrentState());
          
          // 强制赋值：根据关节名字一个一个赋值，防止顺序错误
          // last_point.positions 是数值, last_joint_names 是名字
          if (last_joint_names.size() == last_point.positions.size()) {
              start_state.setVariablePositions(last_joint_names, last_point.positions);
              
              // 应用到 MoveGroup
              move_group.setStartState(start_state);
              RCLCPP_INFO(node->get_logger(), "  ✅ 起点已强制更新 (关节值: %f, %f...)", last_point.positions[0], last_point.positions[1]);
          } else {
              RCLCPP_ERROR(node->get_logger(), "  ❌ 严重错误：保存的关节数据维度不匹配！");
          }

      } else {
          RCLCPP_INFO(node->get_logger(), "  [初始状态] 使用当前传感器读数作为起点");
          move_group.setStartStateToCurrentState();
      }
      // ------------------------------------

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
          RCLCPP_INFO(node->get_logger(), "规划成功，开始执行...");
          move_group.execute(my_plan);
          
          // 【保存数据】提取轨迹中最后时刻的信息
          if (!my_plan.trajectory_.joint_trajectory.points.empty()) {
              last_point = my_plan.trajectory_.joint_trajectory.points.back();
              last_joint_names = my_plan.trajectory_.joint_trajectory.joint_names; // 必须保存名字！
              has_moved_once = true;
          }
      } else {
          RCLCPP_ERROR(node->get_logger(), "规划失败！");
          break;
      }
      
      // 必须有停顿，让 RViz 视觉上跟上
      rclcpp::sleep_for(std::chrono::seconds(1)); 
  }

  rclcpp::shutdown();
  return 0;
}