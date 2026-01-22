#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_debug");
namespace mtc = moveit::task_constructor;

class MTC_Debug_Node {
public:
  MTC_Debug_Node(const rclcpp::NodeOptions& options) 
    : node_{ std::make_shared<rclcpp::Node>("mtc_debug_node", options) } {}
  
  void run() {
    RCLCPP_INFO(LOGGER, "====== MTC调试程序 ======");
    
    // 测试1: 检查机器人模型
    testRobotModel();
    
    // 测试2: 检查末端执行器
    testEndEffector();
    
    // 测试3: 逐步测试各个阶段
    testStagesStepByStep();
  }
  
private:
  void testRobotModel() {
    RCLCPP_INFO(LOGGER, "\n[测试1] 检查机器人模型");
    
    // 创建任务来获取机器人模型
    mtc::Task task;
    task.loadRobotModel(node_);
    
    auto robot_model = task.getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(LOGGER, "无法加载机器人模型");
      return;
    }
    
    RCLCPP_INFO(LOGGER, "机器人模型: %s", robot_model->getName().c_str());
    
    // 检查组
    std::vector<std::string> groups = {"fairino16_v6_group", "gripper"};
    for (const auto& group_name : groups) {
      if (robot_model->hasJointModelGroup(group_name)) {
        auto group = robot_model->getJointModelGroup(group_name);
        RCLCPP_INFO(LOGGER, "✓ 组 '%s' 存在 (%d个关节)", 
                   group_name.c_str(), group->getVariableCount());
      } else {
        RCLCPP_ERROR(LOGGER, "✗ 组 '%s' 不存在", group_name.c_str());
      }
    }
  }
  
  void testEndEffector() {
    RCLCPP_INFO(LOGGER, "\n[测试2] 检查末端执行器");
    
    mtc::Task task;
    task.loadRobotModel(node_);
    
    auto robot_model = task.getRobotModel();
    if (!robot_model) return;
    
    auto eefs = robot_model->getEndEffectors();
    if (eefs.empty()) {
      RCLCPP_ERROR(LOGGER, "没有配置末端执行器！");
      RCLCPP_ERROR(LOGGER, "请在SRDF中添加: <end_effector name=\"gripper\" parent_link=\"wrist3_link\" group=\"gripper\" parent_group=\"fairino16_v6_group\"/>");
    } else {
      for (const auto& eef_pair : eefs) {
        RCLCPP_INFO(LOGGER, "末端执行器: %s", eef_pair.first.c_str());
      }
    }
  }
  
  void testStagesStepByStep() {
    RCLCPP_INFO(LOGGER, "\n[测试3] 逐步测试阶段");
    
    // 测试基本移动
    testBasicMovement();
    
    // 测试夹爪控制
    testGripperControl();
    
    // 测试简单抓取
    testSimplePick();
  }
  
  void testBasicMovement() {
    RCLCPP_INFO(LOGGER, "\n[测试3.1] 基本移动测试");
    
    mtc::Task task;
    task.stages()->setName("基本移动测试");
    task.loadRobotModel(node_);
    
    task.setProperty("group", "fairino16_v6_group");
    
    // 当前状态
    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_current));
    
    // 移动到ready
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto stage_move = std::make_unique<mtc::stages::MoveTo>("move to ready", planner);
    stage_move->setGroup("fairino16_v6_group");
    stage_move->setGoal("ready");
    task.add(std::move(stage_move));
    
    if (runTask(task, "基本移动")) {
      RCLCPP_INFO(LOGGER, "✓ 基本移动测试成功");
    } else {
      RCLCPP_ERROR(LOGGER, "✗ 基本移动测试失败");
    }
  }
  
  void testGripperControl() {
    RCLCPP_INFO(LOGGER, "\n[测试3.2] 夹爪控制测试");
    
    mtc::Task task;
    task.stages()->setName("夹爪控制测试");
    task.loadRobotModel(node_);
    
    task.setProperty("group", "fairino16_v6_group");
    task.setProperty("eef", "gripper");
    
    // 当前状态
    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_current));
    
    // 打开夹爪
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", planner);
    stage_open->setGroup("gripper");
    stage_open->setGoal("open");
    task.add(std::move(stage_open));
    
    // 关闭夹爪
    auto stage_close = std::make_unique<mtc::stages::MoveTo>("close hand", planner);
    stage_close->setGroup("gripper");
    stage_close->setGoal("close");
    task.add(std::move(stage_close));
    
    if (runTask(task, "夹爪控制")) {
      RCLCPP_INFO(LOGGER, "✓ 夹爪控制测试成功");
    } else {
      RCLCPP_ERROR(LOGGER, "✗ 夹爪控制测试失败");
    }
  }
  
  void testSimplePick() {
    RCLCPP_INFO(LOGGER, "\n[测试3.3] 简单抓取测试");
    
    // 先添加物体到场景
    addObjectToScene();
    
    mtc::Task task;
    task.stages()->setName("简单抓取测试");
    task.loadRobotModel(node_);
    
    const auto& arm_group_name = "fairino16_v6_group";
    const auto& hand_group_name = "gripper";
    
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", "gripper_tip_link");
    
    // 当前状态
    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_current));
    
    // 移动到物体附近（简单版本）
    auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto stage_connect = std::make_unique<mtc::stages::Connect>(
        "move near object",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, planner}});
    stage_connect->setTimeout(5.0);
    task.add(std::move(stage_connect));
    
    if (runTask(task, "简单抓取")) {
      RCLCPP_INFO(LOGGER, "✓ 简单抓取测试成功");
    } else {
      RCLCPP_ERROR(LOGGER, "✗ 简单抓取测试失败");
    }
  }
  
  bool runTask(mtc::Task& task, const std::string& test_name) {
    try {
      task.init();
      RCLCPP_INFO(LOGGER, "  %s: 初始化成功", test_name.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "  %s: 初始化失败: %s", test_name.c_str(), e.what());
      return false;
    }
    
    if (task.plan(3)) {
      RCLCPP_INFO(LOGGER, "  %s: 规划成功", test_name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "  %s: 规划失败", test_name.c_str());
      return false;
    }
  }
  
  void addObjectToScene() {
    moveit_msgs::msg::CollisionObject object;
    object.id = "test_object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.3;    // 更近
    pose.position.y = 0.0;    // 正前方
    pose.position.z = 0.05;   // 稍微抬高
    pose.orientation.w = 1.0;
    object.pose = pose;
    
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
    RCLCPP_INFO(LOGGER, "已添加测试物体到场景中");
  }
  
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<MTC_Debug_Node>(options);
  node->run();
  
  rclcpp::shutdown();
  return 0;
}