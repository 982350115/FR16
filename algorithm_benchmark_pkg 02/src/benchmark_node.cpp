/**
 * benchmark_node.cpp
 * 终极融合版：
 * 1. 集成 IK 逆解优化 -> 解决 Pilz PTP 大回环问题
 * 2. 集成 TOTG 时间参数化 -> 解决 RRT/OMPL 执行报错问题
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h> // IK 需要
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h> // TOTG 需要
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

// === 核心修复函数：手动为轨迹添加时间戳 (解决 RRT 不执行的问题) ===
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
    
    // 复位时也建议使用 TOTG 保护，防止复位动作本身报错
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto err = move_group.plan(plan);
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
        // 修复复位轨迹
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

    const std::string PLANNING_GROUP = "fairino16_v6_group";
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, PLANNING_GROUP);

    // 1. 算法列表
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

    RCLCPP_INFO(node->get_logger(), "=== 开始算法对比实验 (IK优化 + TOTG修复) ===");

    // 2. 初始化复位
    if(!go_to_home(move_group)) {
        RCLCPP_ERROR(node->get_logger(), "初始化复位失败！");
        return -1;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 3. 生成目标点 (相对于 Home 的偏移)
    geometry_msgs::msg::Pose home_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> targets;
    auto p1 = home_pose; p1.position.x += 0.3; p1.position.z -= 0.1; targets.push_back(p1);
    auto p2 = p1;        p2.position.y -= 0.3; targets.push_back(p2);
    auto p3 = p2;        p3.position.z -= 0.2; targets.push_back(p3);
    auto p4 = p3;        p4.position.y += 0.3; targets.push_back(p4);
    auto p5 = p4;        p5.position.z += 0.2; targets.push_back(p5);
    auto p6 = home_pose;                       targets.push_back(p6);

    // 4. 循环测试
    for (const auto& algo : algorithms) {
        RCLCPP_INFO(node->get_logger(), "********** 测试算法: %s **********", algo.name.c_str());

        // 切换算法前强制复位
        if (!go_to_home(move_group)) {
            RCLCPP_ERROR(node->get_logger(), "复位失败，跳过该算法");
            continue;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 设置通用参数
        move_group.setPlanningPipelineId(algo.pipeline);
        move_group.setPlannerId(algo.planner_id);
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        
        // 速度比例统一设置 (也会传给 TOTG)
        double vel_scale = 0.5;
        double acc_scale = 0.5;
        move_group.setMaxVelocityScalingFactor(vel_scale);
        move_group.setMaxAccelerationScalingFactor(acc_scale);

        for (size_t i = 0; i < targets.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "--- 前往目标点 [%ld / 6] ---", i+1);

            // >>>>>>>>>>>>>>>>> 修复 1: IK 逆解优化 (解决 PTP 大回环) >>>>>>>>>>>>>>>>>
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            moveit::core::RobotState target_state = *current_state; 
            const moveit::core::JointModelGroup* joint_model_group = 
                current_state->getJointModelGroup(PLANNING_GROUP);

            // 计算 IK (寻找最近解)
            bool found_ik = target_state.setFromIK(joint_model_group, targets[i], 0.1);

            if (found_ik) {
                std::vector<double> target_joints;
                target_state.copyJointGroupPositions(joint_model_group, target_joints);
                move_group.setJointValueTarget(target_joints);
                // RCLCPP_INFO(node->get_logger(), "IK 优化成功");
            } else {
                RCLCPP_WARN(node->get_logger(), "IK 失败，回退到 Pose Target");
                move_group.setPoseTarget(targets[i]);
            }
            // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

            // 规划
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto start_time = std::chrono::high_resolution_clock::now();
            auto err_code = move_group.plan(plan);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            bool success = (err_code == moveit::core::MoveItErrorCode::SUCCESS);
            double planning_duration = std::chrono::duration<double>(end_time - start_time).count();
            double path_len = 0.0;

            if (success) {
                // >>>>>>>>>>>>>>>>> 修复 2: TOTG 时间参数化 (解决 RRT 不执行) >>>>>>>>>>>>>>>>>
                // 只要是 OMPL 产生的轨迹，大概率时间戳有问题，必须修
                if (algo.pipeline == "ompl") {
                    // RCLCPP_INFO(node->get_logger(), "正在应用 TOTG 时间修复...");
                    bool totg_ok = add_time_parameterization(plan.trajectory_, move_group, vel_scale, acc_scale);
                    if (!totg_ok) success = false;
                }
                // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

                if (success) {
                    path_len = calculate_path_length(plan.trajectory_);
                    RCLCPP_INFO(node->get_logger(), "规划成功 | 耗时: %.4fs | 长度: %.4f", planning_duration, path_len);
                    
                    // 执行 (Execute)
                    // 只有这里成功执行了，机械臂才会动，下一个点的起点才正确
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
                 // 失败了就断开，避免乱跑
                 break; 
            }

            // 记录成功数据
            csv_file << algo.name << "," << (i+1) << "," << planning_duration << "," << path_len << ",true\n";
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    csv_file.close();
    RCLCPP_INFO(node->get_logger(), "所有实验结束！");
    rclcpp::shutdown();
    return 0;
}