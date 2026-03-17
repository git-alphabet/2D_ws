#include "rm_behavior_tree/plugins/rmul_2026/action/set_goal_from_center_offset.hpp"

#include <cmath>

#include "behaviortree_ros2/plugins.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

SetGoalFromCenterOffsetAction::SetGoalFromCenterOffsetAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams &)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus SetGoalFromCenterOffsetAction::tick()
{
  auto cx = getInput<double>("center_x");
  auto cy = getInput<double>("center_y");
  auto ox = getInput<double>("offset_x");
  auto oy = getInput<double>("offset_y");

  if (!cx || !cy || !ox || !oy) {
    RCLCPP_WARN(
      rclcpp::get_logger("SetGoalFromCenterOffset"),
      "missing input: center_x/center_y/offset_x/offset_y");
    return BT::NodeStatus::FAILURE;
  }

  double center_yaw = 0.0;
  bool rotate_by_center_yaw = true;
  std::string frame_id = "map";
  getInput("center_yaw", center_yaw);
  getInput("rotate_by_center_yaw", rotate_by_center_yaw);
  getInput("frame_id", frame_id);

  const double offset_x = ox.value();
  const double offset_y = oy.value();

  double gx = cx.value() + offset_x;
  double gy = cy.value() + offset_y;

  if (rotate_by_center_yaw) {
    const double cos_yaw = std::cos(center_yaw);
    const double sin_yaw = std::sin(center_yaw);
    gx = cx.value() + cos_yaw * offset_x - sin_yaw * offset_y;
    gy = cy.value() + sin_yaw * offset_x + cos_yaw * offset_y;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame_id;
  goal.pose.position.x = gx;
  goal.pose.position.y = gy;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;

  setOutput("goal_x", gx);
  setOutput("goal_y", gy);
  setOutput("goal_pose", goal);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::SetGoalFromCenterOffsetAction, "SetGoalFromCenterOffset");
