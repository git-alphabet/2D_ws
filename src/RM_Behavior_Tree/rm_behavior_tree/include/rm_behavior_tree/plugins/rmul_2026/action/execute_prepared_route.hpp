#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__EXECUTE_PREPARED_ROUTE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__EXECUTE_PREPARED_ROUTE_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{

class ExecutePreparedRouteAction : public BT::SyncActionNode
{
public:
  ExecutePreparedRouteAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enable", false, "enable prepared route"),
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("arrive_radius", 0.5, "arrival radius"),
      BT::InputPort<bool>("wpt0_enable", false, "enable waypoint 0"),
      BT::InputPort<double>("wpt0_x", 0.0, "waypoint 0 x"),
      BT::InputPort<double>("wpt0_y", 0.0, "waypoint 0 y"),
      BT::InputPort<bool>("wpt1_enable", false, "enable waypoint 1"),
      BT::InputPort<double>("wpt1_x", 0.0, "waypoint 1 x"),
      BT::InputPort<double>("wpt1_y", 0.0, "waypoint 1 y"),
      BT::InputPort<bool>("wpt2_enable", false, "enable waypoint 2"),
      BT::InputPort<double>("wpt2_x", 0.0, "waypoint 2 x"),
      BT::InputPort<double>("wpt2_y", 0.0, "waypoint 2 y"),
      BT::InputPort<std::string>("frame_id", "map", "goal frame"),
      BT::OutputPort<double>("goal_x"),
      BT::OutputPort<double>("goal_y"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
    };
  }

  BT::NodeStatus tick() override;

private:
  struct Waypoint
  {
    double x{0.0};
    double y{0.0};
  };

  std::vector<Waypoint> waypoints_;
  bool initialized_{false};
  bool completed_{false};
  std::size_t current_index_{0};

  void loadWaypointsFromPorts();
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__EXECUTE_PREPARED_ROUTE_HPP_
