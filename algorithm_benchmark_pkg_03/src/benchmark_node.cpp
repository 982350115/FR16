/**
 * benchmark_node.cpp - Algorithm Benchmark 03
 * 特性：
 * 1. 运行在受限关节空间 (Constrained Joint Space)
 * 2. 集成 IK 逆解优化 (Nearest Solution)
 * 3. 集成 TOTG 时间参数化修复
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>

// 计算轨迹长度
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

// TOTG 修复函数
bool add_time_parameterization(moveit_msgs::msg::RobotTrajectory& trajectory, 
                               moveit::planning_interface::MoveGroupInterface& move_group,
                               double velocity_scaling, 
                               double acceleration_scaling) 
{
    robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool success = totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling);
    if (success) {
        rt.getRobotTrajectoryMsg(trajectory);
        return true;
    }
    return false;
}

// 复位函数
bool go_to_home(moveit::planning_interface::MoveGroupInterface& move_group) {
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    
    // Home: 全 0 位置。注意：如果您限制了关节范围不包含0，这里会报错。
    // 我们设置的 J1 +/- 1.57 包含 0，所以没问题。
    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(home_joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto err = move_group.plan(plan);
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
        if(add_time_parameterization(plan.trajectory_, move_group, 0.5, 0.5)) {
            return (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
    }
    return false;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("benchmark_node_03",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    const std::string PLANNING_GROUP = "fairino16_v6_group";
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, PLANNING_GROUP);

    // 算法列表
    struct AlgoConfig {
        std::string name;
        std::string pipeline;
        std::string planner_id;
    };
    std::vector<AlgoConfig> algorithms = {
        {"RRTConnect", "ompl", "RRTConnectkConfigDefault"},
        {"RRTStar",    "ompl", "RRTstarkConfigDefault"},
        {"Pilz_PTP",   "pilz_industrial_motion_planner", "PTP"}
    };

    std::ofstream csv_file;
    csv_file.open("benchmark_03_constrained_results.csv"); // 输出文件名更改
    csv_file << "Algorithm,TargetID,PlanningTime,PathLength,Success\n";

    RCLCPP_INFO(node->get_logger(), "=== 开始算法对比实验 03 (受限关节空间) ===");

    if(!go_to_home(move_group)) {
        RCLCPP_ERROR(node->get_logger(), "初始化复位失败！请检查 Home 点是否在您的限制范围内。");
        return -1;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 目标点生成
    geometry_msgs::msg::Pose home_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> targets;
    auto p1 = home_pose; p1.position.x += 0.3; p1.position.z -= 0.1; targets.push_back(p1);
    auto p2 = p1;        p2.position.y -= 0.3; targets.push_back(p2);
    auto p3 = p2;        p3.position.z -= 0.2; targets.push_back(p3);
    auto p4 = p3;        p4.position.y += 0.3; targets.push_back(p4); // 注意：这里 y 偏移如果太大，可能触碰 J1 限制
    auto p5 = p4;        p5.position.z += 0.2; targets.push_back(p5);
    auto p6 = home_pose;                       targets.push_back(p6);

    for (const auto& algo : algorithms) {
        RCLCPP_INFO(node->get_logger(), "********** 测试算法: %s **********", algo.name.c_str());

        if (!go_to_home(move_group)) {
            RCLCPP_ERROR(node->get_logger(), "复位失败，跳过");
            continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        move_group.setPlanningPipelineId(algo.pipeline);
        move_group.setPlannerId(algo.planner_id);
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        for (size_t i = 0; i < targets.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "--- 前往目标点 [%ld / 6] ---", i+1);

            // IK 优化
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            moveit::core::RobotState target_state = *current_state; 
            const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);

            // setFromIK 会自动遵守我们在 yaml 中设置的 Joint Limits！
            // 如果目标点在限制范围外，这里会直接返回 false，避免了规划器瞎跑
            bool found_ik = target_state.setFromIK(joint_model_group, targets[i], 0.1);

            if (found_ik) {
                std::vector<double> target_joints;
                target_state.copyJointGroupPositions(joint_model_group, target_joints);
                move_group.setJointValueTarget(target_joints);
            } else {
                RCLCPP_WARN(node->get_logger(), "IK 在限制范围内无解，尝试 Pose Target (可能会失败)");
                move_group.setPoseTarget(targets[i]);
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto start_time = std::chrono::high_resolution_clock::now();
            auto err_code = move_group.plan(plan);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            bool success = (err_code == moveit::core::MoveItErrorCode::SUCCESS);
            double duration = std::chrono::duration<double>(end_time - start_time).count();
            double len = 0.0;

            if (success) {
                if (algo.pipeline == "ompl") {
                    if (!add_time_parameterization(plan.trajectory_, move_group, 0.5, 0.5)) success = false;
                }
                if (success) {
                    len = calculate_path_length(plan.trajectory_);
                    RCLCPP_INFO(node->get_logger(), "成功 | 耗时: %.4f | 长度: %.4f", duration, len);
                    auto exec_ret = move_group.execute(plan);
                    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) success = false;
                }
            } else {
                RCLCPP_ERROR(node->get_logger(), "规划失败 (可能是被关节限制阻挡)");
            }

            if (!success) {
                 csv_file << algo.name << "," << (i+1) << "," << duration << ",0,false\n";
                 break; 
            }
            csv_file << algo.name << "," << (i+1) << "," << duration << "," << len << ",true\n";
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    csv_file.close();
    rclcpp::shutdown();
    return 0;
}