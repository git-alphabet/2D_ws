#include "rm_behavior_tree/rm_behavior_tree.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");
  node->declare_parameter<std::string>(
    "style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/attack_left.xml");

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

  BT::RosNodeParams params_robot_control;
  params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
  params_robot_control.default_port_value = "robot_control";

  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  BT::RosNodeParams params_sentry_follower;
  params_sentry_follower.nh = std::make_shared<rclcpp::Node>("sentry_follower");
  params_sentry_follower.default_port_value = "current_goal";

  BT::RosNodeParams params_nav_control;
  params_nav_control.nh = std::make_shared<rclcpp::Node>("nav_control_cmd");
  params_nav_control.default_port_value = "nav_control_cmd";

 

  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_robot_status",
    "sub_game_status",
    "sub_rfid_status",
    "sub_robot_position",
    "init_blackboard_config",
    "detect_respawn_and_set_recovery",
    "clear_recovery_flag",
    "init_search_timer_if_needed",
    "micro_search_supply_card",
    "wait_and_heal",
    "set_nav_goal_from_config",
    "calibrate_center_anchor",
    "set_goal_from_center_offset",
    "execute_nav2_waypoints",
    "cancel_nav_goal",
    "is_recovery_needed",
    "is_supply_card_detected",
    "is_control_zone_detected",
    "is_within_scope",
    "clear_costmap",
    "is_robot_stuck",
    "is_navigation_stuck",
    "snap_goal_to_free_space",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "is_game_time",
    "is_hp_above",
    "is_hp_below",
    "is_dead",
    "is_status_ok",
    "is_friend_ok",
    "get_current_location",
    "move_around",
    "keep_running",
    "not_arrived",
    "navigate_to_goal",
    "print_message",
    "is_at_nav_goal",
    "is_detect_enemy",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    try {
      RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load msg-update plugin '%s': %s", p.c_str(), e.what());
    }
  }

  for (const auto & p : bt_plugin_libs) {
    try {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load BT plugin '%s': %s", p.c_str(), e.what());
    }
  }

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("nav_control_cmd"), params_nav_control);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("sentry_follower"), params_sentry_follower);

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create behavior tree from '%s': %s", bt_xml_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  std::unique_ptr<BT::Groot2Publisher> publisher;
  try {
    const unsigned port = 2667;
    publisher = std::make_unique<BT::Groot2Publisher>(tree, port);
    RCLCPP_INFO(node->get_logger(), "Groot2Publisher started on port %u", port);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "Failed to start Groot2Publisher on port 2667: %s. Continuing without it.", e.what());
  }

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}