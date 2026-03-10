#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_BLACKBOARD_CONFIG_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_BLACKBOARD_CONFIG_HPP_

#include <cstdint>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

class InitBlackboardConfigAction : public BT::SyncActionNode
{
public:
  InitBlackboardConfigAction(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::uint64_t>("heal_wait_ms"),
      BT::OutputPort<double>("heal_min_ratio"),
      BT::OutputPort<std::uint64_t>("search_timeout_ms"),
      BT::OutputPort<std::uint64_t>("recovery_timeout_ms"),
      BT::OutputPort<double>("supply_goal_x"),
      BT::OutputPort<double>("supply_goal_y"),
      BT::OutputPort<double>("arrive_radius")
    };
  }

  BT::NodeStatus tick() override;

private:
  bool initialized_{false};

  // ====== 纯代码配置区（后续你只改这里） ======
  static constexpr std::uint64_t HEAL_WAIT_MS_DEFAULT        = 2000ULL;
  static constexpr double        HEAL_MIN_RATIO_DEFAULT      = 0.60;
  static constexpr std::uint64_t SEARCH_TIMEOUT_MS_DEFAULT   = 3000ULL;
  static constexpr std::uint64_t RECOVERY_TIMEOUT_MS_DEFAULT = 15000ULL;
  static constexpr double        SUPPLY_GOAL_X_DEFAULT       = 0.21;
  static constexpr double        SUPPLY_GOAL_Y_DEFAULT       = -0.32;
  static constexpr double        ARRIVE_RADIUS_DEFAULT       = 0.6;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_BLACKBOARD_CONFIG_HPP_

/*后续怎么维护/增添（按你要求：只改 hpp/cpp）

比如你想新增 cfg.spiral_radius：

hpp：providedPorts 增加

BT::OutputPort<double>("spiral_radius")


hpp：加一个 constexpr 默认值

static constexpr double SPIRAL_RADIUS_DEFAULT = 0.25;


cpp：tick 里增加 setOutput

double spiral_radius = SPIRAL_RADIUS_DEFAULT;
setOutput("spiral_radius", spiral_radius);


TreeNodesModel + XML 加对应 output_port 映射到 {cfg.spiral_radius}

之后所有节点就能用 {cfg.spiral_radius} 了.*/