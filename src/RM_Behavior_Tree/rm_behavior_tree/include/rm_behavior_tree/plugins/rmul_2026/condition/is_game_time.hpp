#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_GAME_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_GAME_TIME_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断比赛是否已开始
 *
 * 满足以下任一条件即返回 SUCCESS（且一旦触发，永久锁存）:
 *   1. game_progress == 期望阶段 且 剩余时间在区间内（正常路径）
 *   2. 检测到 HP 下降（电控丢包导致 game_progress 未更新的容错）
 *   3. 首次拿到 HP 时若已低于初始满血（默认 400），也判定比赛已开始
 *   4. 缺少 game_status 时，允许仅基于 robot_status 的 HP 判定（可配置）
 *
 * 比赛阶段:
 *   {0, "未开始比赛"}, {1, "准备阶段"}, {2, "十五秒裁判系统自检阶段"},
 *   {3, "五秒倒计时"}, {4, "比赛开始"}, {5, "比赛结算中"}
 */
class IsGameTimeCondition : public BT::SimpleConditionNode
{
public:
  IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkGameStart();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<sp_msgs::msg::RMUL>("message"),
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUL>>("robot_msg"),
      BT::InputPort<bool>("allow_no_game_status_hp_fallback"),
        BT::InputPort<int>("initial_full_hp"),
        BT::InputPort<int>("hp_drop_delta"),
      BT::InputPort<int>("game_progress"), BT::InputPort<int>("lower_remain_time"),
      BT::InputPort<int>("higher_remain_time")};
  }

private:
  bool game_started_{false};   // 锁存：一旦检测到比赛开始，永久为 true
  int  last_hp_{-1};           // 上次血量，-1 表示尚未初始化
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_GAME_TIME_HPP_