/**
 * benchmark_node_04.cpp - Algorithm Benchmark 04
 * 特性：
 * 1. 运行在默认关节空间 (Full Joint Space)
 * 2. 核心特性：基于多重采样与加权代价评估的全局 IK 优选 (Global IK Optimization)
 * 3. 目的：通过软件算法消除“大回环”，而非物理限制
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
#include <random>
#include <limits>

// --- 工具函数：计算轨迹长度 ---
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

// --- 工具函数：TOTG 时间参数化 ---
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

// --- 核心算法：IK 优选策略 ---
/**
 * @brief 通过多重采样寻找最优 IK 解
 * @param current_state 机器人当前状态（参考基准）
 * @param target_pose 目标笛卡尔位姿
 * @param jmg 关节模型组
 * @param samples 采样尝试次数
 * @return std::pair<bool, std::vector<double>> {是否成功, 最优关节角}
 */
std::pair<bool, std::vector<double>> get_optimized_ik_solution(
    moveit::core::RobotStatePtr current_state,
    const geometry_msgs::msg::Pose& target_pose,
    const moveit::core::JointModelGroup* jmg,
    int samples = 50)
{
    // 1. 准备工作
    std::vector<double> current_joints;
    current_state->copyJointGroupPositions(jmg, current_joints);
    
    // 权重定义：前三个关节（根部）权重高，后三个关节（腕部）权重低
    // 目的：宁愿手腕多转一点，也不要根部乱动（消除大回环的关键）
    std::vector<double> weights = {5.0, 5.0, 5.0, 1.0, 1.0, 1.0};
    // 如果关节数不匹配，做个防御性调整
    if (current_joints.size() != weights.size()) {
        weights.assign(current_joints.size(), 1.0);
    }

    // 用于计算 Cost 的 Lambda (加权欧氏距离的平方)
    auto calculate_cost = [&](const std::vector<double>& candidate) -> double {
        double cost = 0.0;
        for (size_t i = 0; i < candidate.size(); ++i) {
            double diff = candidate[i] - current_joints[i];
            cost += weights[i] * (diff * diff);
        }
        return cost;
    };

    // 临时状态用于计算
    moveit::core::RobotState temp_state = *current_state;
    
    std::vector<double> best_solution;
    double min_cost = std::numeric_limits<double>::max();
    bool found_any = false;

    // 2. 多重采样循环
    // 策略：我们不仅尝试当前状态作为种子，还尝试随机状态作为种子
    for (int i = 0; i < samples; ++i) {
        // 对于第 0 次，使用当前状态作为种子（最自然的情况）
        // 对于后续次数，生成随机位置作为种子，探索全局解空间
        if (i > 0) {
            temp_state.setToRandomPositions(jmg);
        } else {
            temp_state = *current_state; 
        }

        // 求解 IK
        // setFromIK 会使用 temp_state 当前的关节值作为种子
        bool found = temp_state.setFromIK(jmg, target_pose, 0.05); // 0.05s timeout per attempt

        if (found) {
            std::vector<double> solution;
            temp_state.copyJointGroupPositions(jmg, solution);
            
            double cost = calculate_cost(solution);
            
            // 更新最优解
            if (cost < min_cost) {
                min_cost = cost;
                best_solution = solution;
                found_any = true;
            }
        }
    }

    return {found_any, best_solution};
}

// --- 复位函数 ---
bool go_to_home(moveit::planning_interface::MoveGroupInterface& move_group) {
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    
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
    auto node = std::make_shared<rclcpp::Node>("benchmark_node_04",
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
    csv_file.open("benchmark_04_optimized_ik_results.csv");
    csv_file << "Algorithm,TargetID,PlanningTime,PathLength,Success\n";

    RCLCPP_INFO(node->get_logger(), "=== 开始算法对比实验 04 (全局IK优选策略) ===");

    if(!go_to_home(move_group)) {
        RCLCPP_ERROR(node->get_logger(), "初始化复位失败！");
        return -1;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 目标点生成 (保持与 03 一致以对比)
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

        move_group.setPlanningPipelineId(algo.pipeline);
        move_group.setPlannerId(algo.planner_id);
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5); // 规划器本身尝试次数
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        for (size_t i = 0; i < targets.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "--- 前往目标点 [%ld / 6] ---", i+1);

            // ==========================================
            // 步骤 1: 使用自定义优选策略计算最佳关节角
            // ==========================================
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);

            auto start_ik = std::chrono::high_resolution_clock::now();
            
            // 调用核心优选函数 (尝试 60 次采样)
            auto [found_ik, target_joints] = get_optimized_ik_solution(
                current_state, targets[i], joint_model_group, 60
            );

            auto end_ik = std::chrono::high_resolution_clock::now();
            double ik_time = std::chrono::duration<double>(end_ik - start_ik).count();
            
            if (found_ik) {
                // 如果找到了优选解，将其设为关节目标
                move_group.setJointValueTarget(target_joints);
                // 打印前三个关节值供调试，看是否避免了大幅度翻转
                RCLCPP_INFO(node->get_logger(), "IK优选完成 (%.3fs) | Target J1: %.2f, J2: %.2f", 
                    ik_time, target_joints[0], target_joints[1]);
            } else {
                RCLCPP_WARN(node->get_logger(), "多重采样 IK 未找到解，尝试 Pose Target (风险较高)");
                move_group.setPoseTarget(targets[i]);
            }

            // ==========================================
            // 步骤 2: 调用规划器进行规划 (PTP/RRT)
            // ==========================================
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto start_plan = std::chrono::high_resolution_clock::now();
            auto err_code = move_group.plan(plan);
            auto end_plan = std::chrono::high_resolution_clock::now();
            
            bool success = (err_code == moveit::core::MoveItErrorCode::SUCCESS);
            double duration = std::chrono::duration<double>(end_plan - start_plan).count();
            double len = 0.0;

            if (success) {
                // 如果是 OMPL，补充 TOTG
                if (algo.pipeline == "ompl") {
                    if (!add_time_parameterization(plan.trajectory_, move_group, 0.5, 0.5)) success = false;
                }
                if (success) {
                    len = calculate_path_length(plan.trajectory_);
                    RCLCPP_INFO(node->get_logger(), "规划成功 | 耗时: %.4f | 长度: %.4f", duration, len);
                    
                    // 执行
                    auto exec_ret = move_group.execute(plan);
                    if (exec_ret != moveit::core::MoveItErrorCode::SUCCESS) success = false;
                }
            } else {
                RCLCPP_ERROR(node->get_logger(), "规划失败");
            }

            if (!success) {
                 csv_file << algo.name << "," << (i+1) << "," << duration << ",0,false\n";
                 // 如果失败，不要 break，尝试下一个点，或者根据需求 break
                 // break; 
            } else {
                 csv_file << algo.name << "," << (i+1) << "," << duration << "," << len << ",true\n";
            }
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    csv_file.close();
    rclcpp::shutdown();
    return 0;
}