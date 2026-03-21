#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_BLACKBOARD_CONFIG_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_BLACKBOARD_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

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
      BT::OutputPort<int>("heal_min_hp"),
      BT::OutputPort<std::uint64_t>("search_timeout_ms"),
      BT::OutputPort<std::uint64_t>("recovery_timeout_ms"),
      BT::OutputPort<double>("supply_goal_x"),
      BT::OutputPort<double>("supply_goal_y"),
      BT::OutputPort<double>("control_zone_goal_x"),
      BT::OutputPort<double>("control_zone_goal_y"),
      BT::OutputPort<double>("arrive_radius")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  bool initialized_{false};

  // ====== 纯代码配置区（保底默认值，优先从参数服务器读取） ======
  static constexpr std::uint64_t HEAL_WAIT_MS_DEFAULT        = 2000ULL;
  static constexpr int           HEAL_MIN_HP_DEFAULT         = 400;
  static constexpr std::uint64_t SEARCH_TIMEOUT_MS_DEFAULT   = 3000ULL;
  static constexpr std::uint64_t RECOVERY_TIMEOUT_MS_DEFAULT = 15000ULL;
  
  // 红方坐标预设
  static constexpr double        RED_SUPPLY_GOAL_X_DEFAULT       = 0.21;
  static constexpr double        RED_SUPPLY_GOAL_Y_DEFAULT       = -0.32;
  static constexpr double        RED_CONTROL_ZONE_X_DEFAULT      = 5.13;
  static constexpr double        RED_CONTROL_ZONE_Y_DEFAULT      = -3.94;

  // 蓝方坐标预设 (根据中心点对称或实测值填写)
  static constexpr double        BLUE_SUPPLY_GOAL_X_DEFAULT      = 7.87;
  static constexpr double        BLUE_SUPPLY_GOAL_Y_DEFAULT      = -4.16;
  static constexpr double        BLUE_CONTROL_ZONE_X_DEFAULT     = 2.95; 
  static constexpr double        BLUE_CONTROL_ZONE_Y_DEFAULT     = -0.54; 

  static constexpr double        ARRIVE_RADIUS_DEFAULT       = 0.6;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_BLACKBOARD_CONFIG_HPP_

