#include "rm_behavior_tree/plugins/rmuc_2026/action/select_objective.hpp"

namespace rm_behavior_tree
{

SelectObjectiveAction::SelectObjectiveAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectObjectiveAction::tick()
{
  int elapsed = 0, remain = 420;
  int hp = 400, hp_max = 400, base_hp = 5000, base_max = 5000, deficit = 500;
  bool outpost_alive = true, base_threat = false;

  getInput("stage_elapsed_time", elapsed);
  getInput("stage_remain_time", remain);
  getInput("hp_cur", hp);
  getInput("hp_max", hp_max);
  getInput("base_hp_cur", base_hp);
  getInput("base_hp_max", base_max);
  getInput("base_deficit_for_fortress", deficit);
  getInput("outpost_alive", outpost_alive);
  getInput("base_threat", base_threat);

  // 策略优先级：
  // 1. 基地受威胁 → 回防 (defend_anchor, 但此处输出为通用目标)
  // 2. 基地掉血超过阈值 → 进攻敌方堡垒 (fortress_enemy)
  // 3. 前哨站存活 → 中心高地
  // 4. 默认 → 梯形高地

  // 注意：实际坐标从 config 黑板读取，此处仅设置 objective_name
  // 上游 XML 会用 objective 选择对应坐标
  std::string objective = "CENTRAL_HIGHLAND";
  double gx = 0, gy = 0;

  if (base_threat) {
    objective = "DEFEND";
  } else if (base_max > 0 && (base_max - base_hp) > deficit) {
    objective = "ENEMY_FORTRESS";
  } else if (outpost_alive) {
    objective = "CENTRAL_HIGHLAND";
  } else {
    objective = "TRAPEZOIDAL_HIGHLAND";
  }

  // TODO: 根据 objective 从黑板配置键 (cfg.xxx_x/y) 获取坐标
  // 目前设置为 0,0；实际由上游子树读取 objective_name 后查表
  setOutput("goal_x", gx);
  setOutput("goal_y", gy);
  setOutput("objective_name", objective);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectObjectiveAction>("SelectObjective");
}
