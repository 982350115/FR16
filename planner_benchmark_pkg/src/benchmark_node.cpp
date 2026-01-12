/**
 * benchmark_node.cpp
 * 修复版：大幅度跨越路径 + Pilz 兼容性增强
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
// 结构体保存结果
struct PlanResult {
    std::string algorithm_name;
    double planning_time;
    double path_length;
    bool success;
};

// 计算轨迹长度（关节空间欧氏距离）
double calculate_path_length(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    double total_length = 0.0;
    const auto& points = trajectory.joint_trajectory.points;
    if (points.size() < 2) return 0.0;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        double dist_sq = 0.0;
        for (size_t j = 0; j < points[i].positions.size(); ++j) {
            double diff = points[i].positions[j] - points[i+1].positions[j];
            dist_sq += diff * diff;
        }
        total_length += std::sqrt(dist_sq);
    }
    return total_length;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("benchmark_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "fairino16_v6_group");

    // === 1. 算法列表 ===
    struct AlgoConfig {
        std::string name;
        std::string pipeline;
        std::string planner_id;
    };

    std::vector<AlgoConfig> algorithms = {
        // RRTConnect: 速度极快，但路径通常很“乱”，会有很多不必要的微小抖动
        {"RRTConnect", "ompl", "RRTConnectkConfigDefault"},
        
        // RRT*: 速度慢，因为它会不断优化路径，试图把线拉直
        {"RRTStar",    "ompl", "RRTstarkConfigDefault"}, 
        
        // Pilz PTP: 工业算法，只要没有障碍物，它规划出的就是理论最短路径（最平滑）
        // 注意：如果 joint_limits.yaml 没配置加速度，这个必挂
        {"Pilz_PTP",   "pilz_industrial_motion_planner", "PTP"} 
    };

    std::ofstream csv_file;
    csv_file.open("benchmark_results.csv"); 
    csv_file << "Algorithm,PlanningTime,PathLength,Success\n";

    RCLCPP_INFO(node->get_logger(), "=== 开始算法对比实验 (大回环测试) ===");

    // === 2. 定义一个容易产生“大回环”的场景 ===
    // 我们不使用“当前位置”做起点，而是强制设置一个起点和一个终点
    
    // 设置起点：所有关节归零，或者一个特定的初始姿态
    // 这里我们用 setStartState 来强制指定起点
    moveit::core::RobotState start_state(*move_group.getCurrentState());
    std::vector<double> start_joints = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0}; // 一个典型的“收起”姿态
    // 注意：如果你的机械臂关节数或限位不同，请根据实际情况调整上述角度
    // 如果不确定，我们还是用当前点做起点，但让当前点先跑到位置 A
    
    // 【策略调整】为了保证实验公平，我们先控制机械臂运动到一个固定的 Start Pose
    RCLCPP_INFO(node->get_logger(), ">>> 正在前往实验起始点...");
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setNamedTarget("home"); // 假设你有 home，如果没有，请注释掉改用下行
    // move_group.setJointValueTarget(start_joints); 
    move_group.move(); // 真的动过去
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 定义目标点 (Pose)：在该位置，机械臂需要伸展
    // 我们设定一个笛卡尔空间的目标，距离起点较远
    geometry_msgs::msg::Pose target_pose = move_group.getCurrentPose().pose;
    target_pose.position.x += 0.3; // 向前伸 30cm
    target_pose.position.y -= 0.4; // 向右移 40cm (大幅度横向移动最容易出乱子)
    target_pose.position.z -= 0.1; // 稍微降低

    // 打印一下目标点信息
    RCLCPP_INFO(node->get_logger(), "目标点: x=%.2f, y=%.2f, z=%.2f", 
        target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // === 3. 循环测试 ===
    for (const auto& algo : algorithms) {
        RCLCPP_INFO(node->get_logger(), "----------------------------------");
        RCLCPP_INFO(node->get_logger(), "正在测试算法: %s ...", algo.name.c_str());

        // 每次规划前，先把“虚拟起点”重置为我们刚才跑到的真实位置
        move_group.setStartStateToCurrentState();
        
        // 设置配置
        move_group.setPlanningPipelineId(algo.pipeline);
        move_group.setPlannerId(algo.planner_id);
        move_group.setPlanningTime(5.0); // 5秒
        move_group.setNumPlanningAttempts(5); // 尝试3次，防止偶然失败
        
        // 关键：设置速度和加速度比例，Pilz 非常依赖这个
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        move_group.setPoseTarget(target_pose);

        // 规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto start_time = node->now();
        
        // 使用 plan() 而不是 move()，我们只规划，不真动，这样起点状态在下一次循环时还是同一个
        auto err_code = move_group.plan(plan);
        
        double duration = (node->now() - start_time).seconds();
        bool success = (err_code == moveit::core::MoveItErrorCode::SUCCESS);

        double path_len = 0.0;
        if (success) {
            path_len = calculate_path_length(plan.trajectory_);
            RCLCPP_INFO(node->get_logger(), "✅ 成功! 耗时: %.4fs, 轨迹长度: %.4f", duration, path_len);
        } else {
            RCLCPP_ERROR(node->get_logger(), "❌ 失败! 错误码: %d", err_code.val);
            if (algo.name == "Pilz_PTP") {
                RCLCPP_WARN(node->get_logger(), "提示: 如果 Pilz 失败，请务必检查 joint_limits.yaml 是否包含 acceleration limits!");
            }
        }

        // 写入数据
        csv_file << algo.name << "," 
                 << duration << "," 
                 << path_len << "," 
                 << (success ? "true" : "false") << "\n";
                 
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    csv_file.close();
    RCLCPP_INFO(node->get_logger(), "实验结束！请运行 python 脚本绘图。");

    rclcpp::shutdown();
    return 0;
}