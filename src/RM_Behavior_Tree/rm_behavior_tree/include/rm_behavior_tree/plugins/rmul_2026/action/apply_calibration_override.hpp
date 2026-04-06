#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__APPLY_CALIBRATION_OVERRIDE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__APPLY_CALIBRATION_OVERRIDE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class ApplyCalibrationOverrideAction : public BT::SyncActionNode
{
public:
  ApplyCalibrationOverrideAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      // 控制区标定输入
      BT::InputPort<bool>("control_valid", false, "控制区标定是否有效"),
      BT::InputPort<double>("control_x", 0.0, "标定控制区 x"),
      BT::InputPort<double>("control_y", 0.0, "标定控制区 y"),
      // 补给区标定输入
      BT::InputPort<bool>("supply_valid", false, "补给区标定是否有效"),
      BT::InputPort<double>("supply_x", 0.0, "标定补给区 x"),
      BT::InputPort<double>("supply_y", 0.0, "标定补给区 y"),
      // 覆盖目标（输出到 cfg.* 黑板键）
      BT::OutputPort<double>("control_zone_goal_x"),
      BT::OutputPort<double>("control_zone_goal_y"),
      BT::OutputPort<double>("supply_goal_x"),
      BT::OutputPort<double>("supply_goal_y"),
    };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  bool control_logged_{false};
  bool supply_logged_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__APPLY_CALIBRATION_OVERRIDE_HPP_
