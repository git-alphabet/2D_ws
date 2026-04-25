#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_DETECT_ENEMY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_DETECT_ENEMY_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{

class RmucIsDetectEnemyCondition : public BT::SimpleConditionNode
{
public:
  RmucIsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkDetectEnemy();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("message")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_DETECT_ENEMY_HPP_
