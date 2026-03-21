#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_AND_HEAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_AND_HEAL_HPP_

#include <cstdint>
#include <string>

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

class WaitAndHealAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUL>
{
public:
  WaitAndHealAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      // 订阅 RMUL
      BT::InputPort<std::string>("topic_name"),

      // 兼容现有 XML：保留端口
      BT::InputPort<std::uint64_t>("now_ms"),
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),

      // 真正使用的端口
      BT::BidirectionalPort<std::uint64_t>("heal_start_ms"),
      BT::InputPort<std::uint64_t>("heal_wait_ms"),
      BT::InputPort<int>("heal_min_hp")
    };
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUL> & last_msg) override;

private:
  static constexpr int MAX_HP_FIXED = 400;
  bool has_hp_cache_ = false;
  int last_hp_cache_ = 0;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_AND_HEAL_HPP_
