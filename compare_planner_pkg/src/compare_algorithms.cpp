#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

// ==================== 【核心修复】暴力清洗轨迹 ====================
// 强制重写时间戳，确保控制器绝对不会拒绝
void sanitize_trajectory(moveit_msgs::msg::RobotTrajectory& trajectory) {
    if (trajectory.joint_trajectory.points.empty()) return;

    auto& points = trajectory.joint_trajectory.points;
    double time_counter = 0.0;

    // 假设我们希望每两个点之间间隔 0.04秒 (25Hz)
    // 这个频率对于仿真来说很安全
    const double dt = 0.04; 

    for (size_t i = 0; i < points.size(); ++i) {
        if (i == 0) {
            time_counter = 0.0;
        } else {
            time_counter += dt;
        }

        int32_t sec = static_cast<int32_t>(time_counter);
        uint32_t nanosec = static_cast<uint32_t>((time_counter - sec) * 1e9);

        points[i].time_from_start.sec = sec;
        points[i].time_from_start.nanosec = nanosec;

        // 清空速度和加速度，让控制器自己去计算插值
        // 这能避免“速度限制超标”的报错
        points[i].velocities.clear();
        points[i].accelerations.clear();
    }
    
    printf(">>> [DEBUG] 轨迹已清洗: 点数=%zu, 总时长=%.2fs\n", points.size(), time_counter);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("compare_algorithms_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "fairino16_v6_group");

  // 1. 算法设置
  // std::string planner_id = "RRTConnectkConfigDefault"; // 容易大回环
  std::string planner_id = "RRTstarkConfigDefault";    // 路径平滑 (RRT*)

  RCLCPP_INFO(node->get_logger(), "当前使用的算法: %s", planner_id.c_str());

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId(planner_id);
  move_group.setPlanningTime(5.0); 
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 2. 定义目标点
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

  // 3. 循环执行
  for (size_t i = 0; i < target_poses.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "---------------------------------------");
      RCLCPP_INFO(node->get_logger(), ">>> 准备前往目标点 %zu ...", i+1);
      
      move_group.setPoseTarget(target_poses[i]);
      
      // 每次都更新起点为当前状态 (如果上一步执行成功，这里就是正确的)
      move_group.setStartStateToCurrentState();

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
          // 【关键调用】在执行前，强制清洗轨迹
          sanitize_trajectory(my_plan.trajectory_);

          RCLCPP_INFO(node->get_logger(), "规划成功，开始执行...");
          
          // 执行轨迹
          auto err = move_group.execute(my_plan);
          
          if (err == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(node->get_logger(), "✅ 执行成功！机器人应该已经到达目标点 %zu", i+1);
          } else {
              RCLCPP_ERROR(node->get_logger(), "❌ 执行失败！错误代码: %d", err.val);
          }

      } else {
          RCLCPP_ERROR(node->get_logger(), "❌ 规划失败！");
          break;
      }
      
      // 停顿2秒，确保动作完成且肉眼可见
      rclcpp::sleep_for(std::chrono::seconds(2)); 
  }

  rclcpp::shutdown();
  return 0;
}