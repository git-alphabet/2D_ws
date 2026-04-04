#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__KEEP_RUNNING_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__KEEP_RUNNING_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

/**
 * @brief Action 节点：永远返回 RUNNING
 *
 * 设计目的：
 *  - 用于“阻塞型”分支
 *  - 常配合条件节点使用（例如 NotArrived + KeepRunning）
 *
 * 特点：
 *  - 不访问黑板
 *  - 不依赖 ROS
 *  - 不维护任何内部状态
 */
class KeepRunningAction : public BT::StatefulActionNode
{
public:
  KeepRunningAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__KEEP_RUNNING_HPP_
