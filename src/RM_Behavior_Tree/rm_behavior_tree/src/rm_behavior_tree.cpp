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
    "is_navigation_stuck",
    "is_in_semantic_zone",
    "snap_goal_to_free_space",
    "find_escape_point",
    "is_goal_area_clear",
    "is_nav_goal_rejected",
    "find_approach_point",
    "apply_calibration_override",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "is_game_time",
    "is_hp_above",
    "is_hp_below",
    "is_hp_increasing",
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

  // ─── RMUC 2026 参数注入 ───
  // 从 ROS 参数 (rmuc_sentry_config.*) 读取，注入到树的根黑板
  // InitSentryConfig 的 getInput 会优先读取黑板中已有的值
  {
    auto bb = tree.rootBlackboard();
    // 坐标参数
    const std::vector<std::string> coord_keys = {
      "home_x","home_y","supply_zone_x","supply_zone_y",
      "base_buff_x","base_buff_y","outpost_buff_x","outpost_buff_y",
      "fortress_ally_x","fortress_ally_y","fortress_enemy_x","fortress_enemy_y",
      "central_highland_x","central_highland_y",
      "ladder_highland_x","ladder_highland_y",
      "defend_anchor_x","defend_anchor_y",
      "patrol_wpt_0_x","patrol_wpt_0_y",
      "patrol_wpt_1_x","patrol_wpt_1_y",
      "patrol_wpt_2_x","patrol_wpt_2_y"
    };
    // double 阈值参数
    const std::vector<std::pair<std::string, double>> double_keys = {
      {"arrive_radius", 0.35}, {"enemy_near_base_radius", 2.0}
    };
    // int 阈值参数
    const std::vector<std::pair<std::string, int>> int_keys = {
      {"hp_low", 180}, {"hp_safe", 280},
      {"heat_high", 210},
      {"ammo_low", 80}, {"ammo_target", 300},
      {"base_deficit_for_fortress", 500},
      {"objective_hold_ms", 12000},
      {"combat_fire_burst_ms", 180}, {"combat_fire_pause_ms", 120}
    };

    const std::string prefix = "rmuc_sentry_config.";
    int injected = 0;

    for (const auto & k : coord_keys) {
      auto param_name = prefix + k;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<double>(param_name, 0.0);
      }
      double v = node->get_parameter(param_name).as_double();
      if (v != 0.0) {
        bb->set("cfg." + k, v);
        injected++;
      }
    }
    for (const auto & [k, def] : double_keys) {
      auto param_name = prefix + k;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<double>(param_name, def);
      }
      double v = node->get_parameter(param_name).as_double();
      bb->set("cfg." + k, v);
      injected++;
    }
    for (const auto & [k, def] : int_keys) {
      auto param_name = prefix + k;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<int>(param_name, def);
      }
      int v = static_cast<int>(node->get_parameter(param_name).as_int());
      bb->set("cfg." + k, v);
      injected++;
    }
    RCLCPP_INFO(node->get_logger(), "Injected %d RMUC config params into blackboard", injected);
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