#include "rm_behavior_tree/plugins/rmul_2026/condition/not_arrived.hpp"

namespace rm_behavior_tree
{

NotArrived::NotArrived(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  // 纯判断节点：不依赖 ROS node / logger 等资源
}

BT::NodeStatus NotArrived::tick()
{
  // ------------------------------------------------
  // 1) 从黑板读取 RFID.msg
  // ------------------------------------------------
  sp_msgs::msg::RMUL rfid_msg;
  auto res = getInput<sp_msgs::msg::RMUL>("rfid_status");
  if (!res) {
    // 黑板没有该 key / 类型不匹配 / 尚未写入：保守失败
    // （如需排障，可在此处加你们自己的日志输出）
    return BT::NodeStatus::FAILURE;
  }
  rfid_msg = res.value();

  // ------------------------------------------------
  // 2) 提取“是否已刷到补给区交互”的语义量
  // ------------------------------------------------
  const bool arrived = rfid_msg.rfid_supply_arrived;

  // ------------------------------------------------
  // 3) 未到达 -> SUCCESS；已到达 -> FAILURE
  // ------------------------------------------------
  return arrived ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::NotArrived>("NotArrived");
}
