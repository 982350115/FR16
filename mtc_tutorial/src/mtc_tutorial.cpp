/**
 * MTC (MoveIt Task Constructor) 基础教学代码 - 修正版
 * 适用场景：Fairino16 机械臂抓取圆柱体
 * 修正内容：优化了物体坐标、抓取姿态计算和防碰撞参数
 */

// ==================== 1. 必要的头文件 ====================
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// 处理几何消息和 TF 转换的兼容性写法（自动判断 ROS 版本）
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// ==================== 2. 全局定义 ====================
// 定义一个日志记录器，方便在终端打印信息
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

// 给 moveit::task_constructor 起个短名字叫 mtc，少打几个字
namespace mtc = moveit::task_constructor;

// ==================== 3. 类定义 ====================
class MTCTaskNode
{
public:
  // 构造函数
  MTCTaskNode(const rclcpp::NodeOptions& options);

  // 获取节点接口（用于多线程执行器）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  // 核心功能：执行任务
  void doTask();

  // 核心功能：生成场景（桌子、圆柱体）
  void setupPlanningScene();

private:
  // 内部函数：构建任务流程图
  mtc::Task createTask();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_; // 智能指针管理的 ROS 节点
};

// ==================== 4. 函数实现 ====================

// 获取节点基础接口
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// 构造函数初始化
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

/**
 * [关键函数] 设置规划场景
 * 这里定义了我们要抓的物体在哪里。
 * 之前的错误往往是因为物体设置得太远或者陷在地下。
 */
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";            // 给物体起个名字
  object.header.frame_id = "world"; // 参考世界坐标系

  // 定义物体形状：高 0.1m, 半径 0.02m 的圆柱体
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  // 定义物体位置
  geometry_msgs::msg::Pose pose;
  // 【修正1】拉近物体：从 0.5 改为 0.4，确保机械臂够得着
  pose.position.x = 0.4; 
  // 【修正2】居中放置：从 -0.25 改为 0.0，避免侧面抓取时关节限位
  pose.position.y = 0.0; 
  // 【修正3】放在桌面上：圆柱体高0.1，中心在0.05，这样底部正好贴着桌面(z=0)
  pose.position.z = 0.05; 
  pose.orientation.w = 1.0; 
  object.pose = pose;

  // 调用 MoveIt 接口添加物体
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

/**
 * [核心流程] 执行任务
 */
void MTCTaskNode::doTask()
{
  task_ = createTask(); // 1. 构建任务树

  try
  {
    RCLCPP_INFO(LOGGER, "正在初始化任务...");
    task_.init(); // 2. 初始化检查
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "任务初始化失败: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, "开始规划路径 (请关注 RViz 中的 Motion Planning Tasks 面板)...");
  
  // 3. 开始规划 (尝试 5 次)
  // 如果这里返回 false，说明找不到解
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "任务规划失败！请检查 RViz 中的红色阶段。");
    return;
  }

  // 4. 发布结果给 RViz 显示动画
  // 只有规划成功了，这一步才会执行，你才能在 RViz 点击 demo task 看到动画
  RCLCPP_INFO(LOGGER, "规划成功！正在发布轨迹...");
  task_.introspection().publishSolution(*task_.solutions().front());

  // 5. 让真机/仿真模型动起来 (Execute)
  RCLCPP_INFO(LOGGER, "开始执行轨迹...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "任务执行失败");
    return;
  }

  RCLCPP_INFO(LOGGER, "任务圆满完成！");
}

/**
 * [最复杂的部分] 构建任务的具体步骤
 */
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  // ---------- 配置机器人参数 (必须与 SRDF 文件一致) ----------
  const auto& arm_group_name = "fairino16_v6_group"; // 机械臂组名
  const auto& hand_group_name = "gripper";           // 夹爪组名
  const auto& hand_frame = "gripper_tip_link";       // 夹爪末端坐标系(IK计算的目标)

  // 设置全局属性
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;

  // ---------------------------------------------------------
  // 阶段 1: 获取当前状态
  // ---------------------------------------------------------
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // 定义三种规划器
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_); // 采样规划(避障好)
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>(); // 关节插值(简单动作)
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>(); // 笛卡尔(直线运动)
  
  // 调慢一点速度，运动更稳
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.01);

  // ---------------------------------------------------------
  // 阶段 2: 张开夹爪
  // ---------------------------------------------------------
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open"); // 对应 SRDF 中的姿态名
  task.add(std::move(stage_open_hand));

  // ---------------------------------------------------------
  // 阶段 3: 移动到抓取点 (Connect)
  // ---------------------------------------------------------
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(10.0); // 给足计算时间
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // ---------------------------------------------------------
  // 阶段 4: 抓取流程容器 (Pick Object)
  // ---------------------------------------------------------
  mtc::Stage* attach_object_stage = nullptr;
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    // 将主任务属性传递给子容器
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 4.1 接近物体 (Approach)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      // 【修正4】放宽最小距离，防止因距离太近报错
      stage->setMinMaxDistance(0.01, 0.15);

      // 设置接近方向：沿手部 Z 轴向前
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0; 
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 4.2 生成抓取姿态 (Generate Grasp Pose)
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12); // 每 15 度生成一个候选姿态
      stage->setMonitoredStage(current_state_ptr);

      // 【关键修正5】定义抓取时的手部姿态
      Eigen::Isometry3d grasp_frame_transform;
      grasp_frame_transform.setIdentity();

      // 旋转：绕 X 轴转 180 度，让夹爪 Z 轴垂直向下 (假设原 Z 轴向前)
      // 这是一个标准的顶部抓取姿态
      // 方法：使用构造函数直接初始化
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      grasp_frame_transform.linear() = q.matrix();

      // 平移：抓取点相对于物体中心的偏移
      // 物体高 0.1m (中心在 0.05m)。设置 z=0.08 意味着夹爪尖端在物体中心上方 8cm 处
      // 这样手指大概能包住物体的上部，且不会碰到桌子
      grasp_frame_transform.translation().z() = 0.08; 

      // 4.3 计算逆运动学 (Compute IK)
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8); // 多试几个解
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // 4.4 允许碰撞 (Allow Collision) - 因为手指要接触物体
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    // 4.5 闭合夹爪 (Close Hand)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    // 4.6 附着物体 (Attach Object) - 告诉 ROS 物体现在连在手上了
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // 4.7 举起物体 (Lift Object)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.2);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // 方向：世界坐标系 Z 轴向上
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ---------------------------------------------------------
  // 阶段 5: 移动到放置点 (Connect)
  // ---------------------------------------------------------
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(10.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  // ---------------------------------------------------------
  // 阶段 6: 放置流程容器 (Place Object)
  // ---------------------------------------------------------
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 6.1 生成放置姿态
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      // 【修正6】放置目标：y=0.4 (比原来的 0.5 近一点，确保能放到)
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = 0.4;
      target_pose_msg.pose.position.y = 0.4; 
      target_pose_msg.pose.position.z = 0.05; // 放回桌面上
      target_pose_msg.pose.orientation.w = 1.0; 
      stage->setPose(target_pose_msg);
      
      stage->setMonitoredStage(attach_object_stage);

      // 计算放置逆运动学
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object"); // 放置时对齐的是物体坐标系
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    // 6.2 张开夹爪
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 6.3 恢复碰撞检测
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    // 6.4 分离物体
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    // 6.5 撤退 (Retreat)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // 撤退方向：世界坐标系 X 轴负方向
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.2; 
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  // ---------------------------------------------------------
  // 阶段 7: 回到初始状态
  // ---------------------------------------------------------
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

// ==================== 5. 主函数 ====================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}