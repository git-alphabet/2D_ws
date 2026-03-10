#include "rm_behavior_tree/plugins/rmul_2026/action/set_nav_goal_from_config.hpp"

#include "rclcpp/rclcpp.hpp"   // rclcpp::get_logger
#include "rclcpp/logging.hpp"

namespace rm_behavior_tree
{

SetNavGoalFromConfigAction::SetNavGoalFromConfigAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & /*params*/)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus SetNavGoalFromConfigAction::tick()
{
  // 1. 读取 cfg_x / cfg_y
  auto res_x = getInput<double>("cfg_x");
  auto res_y = getInput<double>("cfg_y");

  const bool has_x = static_cast<bool>(res_x);
  const bool has_y = static_cast<bool>(res_y);

  // 2. 输入缺失：默认不更新输出，避免把车送去 (0,0)
  if (!has_x || !has_y) {
    if (ALLOW_FALLBACK_TO_ZERO && !last_goal_valid_) {
      // 兜底：第一次就缺参且允许 fallback 才写 0,0（默认关闭）
      const double gx = 0.0;
      const double gy = 0.0;
      setOutput("goal_x", gx);
      setOutput("goal_y", gy);
      setOutput("goal_pose", makePose(gx, gy));

      last_goal_valid_ = true;
      last_goal_x_ = gx;
      last_goal_y_ = gy;
    } else {
      // 保守：不写 output，保持黑板旧值（或等待上游先初始化）
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("SetNavGoalFromConfig"),
        *rclcpp::Clock::make_shared(), 2000,
        "cfg_x/cfg_y 缺失：本次不更新 goal，避免写入危险默认值");
    }
    return BT::NodeStatus::SUCCESS;
  }

  double cfg_x = res_x.value();
  double cfg_y = res_y.value();

  // 3. 合法性检查：NaN/inf 则不更新（若已有 last_goal，则继续用 last_goal）
  if (!isFinite(cfg_x) || !isFinite(cfg_y)) {
    if (last_goal_valid_) {
      // 继续沿用上一次有效目标，避免异常值拉飞
      setOutput("goal_x", last_goal_x_);
      setOutput("goal_y", last_goal_y_);
      setOutput("goal_pose", makePose(last_goal_x_, last_goal_y_));

      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("SetNavGoalFromConfig"),
        *rclcpp::Clock::make_shared(), 2000,
        "cfg_x/cfg_y 非法（NaN/inf）：沿用上一次有效 goal");
    } else if (ALLOW_FALLBACK_TO_ZERO) {
      // 没有历史目标且允许 fallback 才写 0,0
      const double gx = 0.0;
      const double gy = 0.0;
      setOutput("goal_x", gx);
      setOutput("goal_y", gy);
      setOutput("goal_pose", makePose(gx, gy));
      last_goal_valid_ = true;
      last_goal_x_ = gx;
      last_goal_y_ = gy;

      RCLCPP_WARN(rclcpp::get_logger("SetNavGoalFromConfig"),
        "cfg_x/cfg_y 非法且无历史 goal：使用兜底 0,0（注意风险）");
    } else {
      // 没历史目标也不允许 fallback：不更新
      RCLCPP_WARN(rclcpp::get_logger("SetNavGoalFromConfig"),
        "cfg_x/cfg_y 非法且无历史 goal：本次不更新");
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 4. 安全限幅：防止配置异常写入极端值
  const double goal_x = clampToMap(cfg_x);
  const double goal_y = clampToMap(cfg_y);

  // 5. 写入黑板：goal_x/goal_y/goal_pose
  setOutput("goal_x", goal_x);
  setOutput("goal_y", goal_y);
  setOutput("goal_pose", makePose(goal_x, goal_y));

  // 6. 更新内部缓存
  last_goal_valid_ = true;
  last_goal_x_ = goal_x;
  last_goal_y_ = goal_y;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SetNavGoalFromConfigAction, "SetNavGoalFromConfig");
