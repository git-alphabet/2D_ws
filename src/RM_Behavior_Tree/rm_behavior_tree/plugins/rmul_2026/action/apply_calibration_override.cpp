#include "rm_behavior_tree/plugins/rmul_2026/action/apply_calibration_override.hpp"

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

ApplyCalibrationOverrideAction::ApplyCalibrationOverrideAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh;
}

BT::NodeStatus ApplyCalibrationOverrideAction::tick()
{
  // 控制区标定覆盖
  auto control_valid = getInput<bool>("control_valid").value_or(false);
  if (control_valid) {
    auto cx = getInput<double>("control_x");
    auto cy = getInput<double>("control_y");
    if (cx && cy) {
      setOutput("control_zone_goal_x", cx.value());
      setOutput("control_zone_goal_y", cy.value());
      if (!control_logged_) {
        RCLCPP_WARN(node_->get_logger(),
          "[CALIB_OVERRIDE] 控制区坐标已覆盖为标定值: (%.3f, %.3f)", cx.value(), cy.value());
        control_logged_ = true;
      }
    }
  }

  // 补给区标定覆盖
  auto supply_valid = getInput<bool>("supply_valid").value_or(false);
  if (supply_valid) {
    auto sx = getInput<double>("supply_x");
    auto sy = getInput<double>("supply_y");
    if (sx && sy) {
      setOutput("supply_goal_x", sx.value());
      setOutput("supply_goal_y", sy.value());
      if (!supply_logged_) {
        RCLCPP_WARN(node_->get_logger(),
          "[CALIB_OVERRIDE] 补给区坐标已覆盖为标定值: (%.3f, %.3f)", sx.value(), sy.value());
        supply_logged_ = true;
      }
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::ApplyCalibrationOverrideAction, "ApplyCalibrationOverride");
