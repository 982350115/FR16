/**
 * benchmark_node.cpp
 * 终极修复版：在代码层强制加入 TOTG 时间参数化，彻底解决 0 时间戳问题
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h> // <--- 关键头文件
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

// === 核心修复函数：手动为轨迹添加时间戳 ===
bool add_time_parameterization(moveit_msgs::msg::RobotTrajectory& trajectory, 
                               moveit::planning_interface::MoveGroupInterface& move_group,
                               double velocity_scaling, 
                               double acceleration_scaling) 
{
    // 1. 将 ROS 消息转换为 RobotTrajectory 对象
    robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), move_group.getName());
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    // 2. 使用 TOTG 算法计算时间戳
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    // 参数：轨迹，最大速度缩放，最大加速度缩放
    bool success = totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling);

    if (success) {
        // 3. 将计算结果转换回 ROS 消息
        rt.getRobotTrajectoryMsg(trajectory);
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("time_param"), "TOTG 时间参数化失败！");
        return false;
    }
}

// 强制复位函数
bool go_to_home(moveit::planning_interface::MoveGroupInterface& move_group) {
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(home_joints);
    
    // 复位时也建议加上手动时间参数化，防止复位动作本身报错
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
    auto node = std::make_shared<rclcpp::Node>("benchmark_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // 确认你的规划组名称
    const std::string PLANNING_GROUP = "fairino16_v6_group"; 
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, PLANNING_GROUP);

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
    csv_file.open("benchmark_final_results.csv");
    csv_file << "Algorithm,TargetID,PlanningTime,PathLength,Success\n";

    RCLCPP_INFO(node->get_logger(), "=== 开始算法对比实验 (含 TOTG 修复) ===");

    // 初始化复位
    if(!go_to_home(move_group)) {
        RCLCPP_ERROR(node->get_logger(), "初始化复位失败！");
        return -1;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 生成目标点
    geometry_msgs::msg::Pose home_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> targets;
    auto p1 = home_pose; p1.position.x += 0.3; p1.position.z -= 0.1; targets.push_back(p1);
    auto p2 = p1;        p2.position.y -= 0.3; targets.push_back(p2);
    auto p3 = p2;        p3.position.z -= 0.2; targets.push_back(p3);
    auto p4 = p3;        p4.position.y += 0.3; targets.push_back(p4);
    auto p5 = p4;        p5.position.z += 0.2; targets.push_back(p5);
    auto p6 = home_pose;                       targets.push_back(p6);

    for (const auto& algo : algorithms) {
        RCLCPP_INFO(node->get_logger(), "********** 测试算法: %s **********", algo.name.c_str());

        if (!go_to_home(move_group)) {
            RCLCPP_ERROR(node->get_logger(), "复位失败，跳过");
            continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 设置参数
        move_group.setPlanningPipelineId(algo.pipeline);
        move_group.setPlannerId(algo.planner_id);
        move_group.setPlanningTime(5.0);
        
        // 速度比例统一设置 (也会传给 TOTG)
        double vel_scale = 0.5;
        double acc_scale = 0.5;
        move_group.setMaxVelocityScalingFactor(vel_scale);
        move_group.setMaxAccelerationScalingFactor(acc_scale);

        for (size_t i = 0; i < targets.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "--- 前往目标点 [%ld / 6] ---", i+1);

            // IK 优化
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            moveit::core::RobotState target_state = *current_state;
            const moveit::core::JointModelGroup* joint_model_group = 
                current_state->getJointModelGroup(PLANNING_GROUP);

            bool found_ik = target_state.setFromIK(joint_model_group, targets[i], 0.1);
            if (found_ik) {
                std::vector<double> target_joints;
                target_state.copyJointGroupPositions(joint_model_group, target_joints);
                move_group.setJointValueTarget(target_joints);
            } else {
                move_group.setPoseTarget(targets[i]);
            }

            // 规划
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto start_time = std::chrono::high_resolution_clock::now();
            auto err_code = move_group.plan(plan);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            bool success = (err_code == moveit::core::MoveItErrorCode::SUCCESS);
            double planning_duration = std::chrono::duration<double>(end_time - start_time).count();
            double path_len = 0.0;

            if (success) {
                // >>>>> 关键修复步骤 <<<<<
                // 对于 RRT/OMPL 算法，强制重新计算时间戳
                // Pilz 算法自带很好的时间戳，但再算一次也无妨（或者可以判断 algo.pipeline == "ompl" 才加）
                if (algo.pipeline == "ompl") {
                    RCLCPP_INFO(node->get_logger(), "正在为 OMPL 轨迹添加时间参数...");
                    bool totg_ok = add_time_parameterization(plan.trajectory_, move_group, vel_scale, acc_scale);
                    if (!totg_ok) success = false;
                }
                // >>>>> 修复结束 <<<<<

                if (success) {
                    path_len = calculate_path_length(plan.trajectory_);
                    RCLCPP_INFO(node->get_logger(), "规划成功 | 耗时: %.4fs | 长度: %.4f", planning_duration, path_len);
                    
                    // 执行 (Execute)
                    auto exec_ret = move_group.execute(plan);
                    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_ERROR(node->get_logger(), "执行失败! 错误码: %d", exec_ret.val);
                        success = false; 
                    }
                }
            } else {
                RCLCPP_ERROR(node->get_logger(), "规划失败!");
            }

            if (!success) {
                 csv_file << algo.name << "," << (i+1) << "," << planning_duration << ",0,false\n";
                 break; 
            }

            csv_file << algo.name << "," << (i+1) << "," << planning_duration << "," << path_len << ",true\n";
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    csv_file.close();
    rclcpp::shutdown();
    return 0;
}