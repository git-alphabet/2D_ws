/**
 * @file rm_behavior_tree_rmuc.cpp
 * @brief RMUC 2026 哨兵行为树 —— 独立入口
 *
 * 与 RMUL 版本 (rm_behavior_tree.cpp) 完全分离：
 *   - RMUC.msg 已拆分为 8 个独立小话题，每类数据走独立话题
 *   - Groot2 使用不同端口 (1668) 以便同时调试
 *
 * 话题约定 (输入 — 订阅):
 *   /game_status       — RMUCGameStatus      (1 Hz)
 *   /robot_status      — RMUCRobotStatus     (10 Hz)
 *   /rfid_status       — RMUCRFIDStatus      (事件驱动)
 *   /robot_position    — RMUCRobotPosition   (50 Hz)
 *   /radar/enemy_tracks— RMUCEnemyTracks     (10-30 Hz)
 *
 * 话题约定 (输出 — 发布):
 *   /sentry_cmd        — RMUCSentryCmd       (2 Hz)
 *   /robot_control     — RMUCRobotControl    (10 Hz)
 *   /nav_control_cmd   — RMUCNavControlCmd   (按需)
 *
 * 共享话题:
 *   goal_pose          — SendGoal (PoseStamped，与 RMUL 共享)
 */

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
  auto node = std::make_shared<rclcpp::Node>("rmuc_behavior_tree");
  node->declare_parameter<std::string>(
    "style", "./config/rmuc_2026/rmuc_2026.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./config/rmuc_2026/rmuc_2026.xml");

  std::cout << "Start RMUC_Behavior_Tree (2026)" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  // ═══════════════════════ ROS Node Params ═══════════════════════
  // 每类消息对应独立的 RosNodeParams，话题名在 default_port_value 中指定

  // ── 输入话题（订阅者） ──
  BT::RosNodeParams params_game_status;
  params_game_status.nh = std::make_shared<rclcpp::Node>("rmuc_game_status_io");
  params_game_status.default_port_value = "/game_status";

  BT::RosNodeParams params_robot_status;
  params_robot_status.nh = std::make_shared<rclcpp::Node>("rmuc_robot_status_io");
  params_robot_status.default_port_value = "/robot_status";

  BT::RosNodeParams params_rfid_status;
  params_rfid_status.nh = std::make_shared<rclcpp::Node>("rmuc_rfid_status_io");
  params_rfid_status.default_port_value = "/rfid_status";

  BT::RosNodeParams params_robot_position;
  params_robot_position.nh = std::make_shared<rclcpp::Node>("rmuc_robot_position_io");
  params_robot_position.default_port_value = "/robot_position";

  BT::RosNodeParams params_radar;
  params_radar.nh = std::make_shared<rclcpp::Node>("rmuc_radar_io");
  params_radar.default_port_value = "/radar/enemy_tracks";

  // ── 输出话题（发布者） ──
  BT::RosNodeParams params_sentry_cmd;
  params_sentry_cmd.nh = std::make_shared<rclcpp::Node>("rmuc_sentry_cmd_io");
  params_sentry_cmd.default_port_value = "/sentry_cmd";

  BT::RosNodeParams params_robot_ctrl;
  params_robot_ctrl.nh = std::make_shared<rclcpp::Node>("rmuc_robot_ctrl_io");
  params_robot_ctrl.default_port_value = "/robot_control";

  BT::RosNodeParams params_nav_cmd;
  params_nav_cmd.nh = std::make_shared<rclcpp::Node>("rmuc_nav_cmd_io");
  params_nav_cmd.default_port_value = "/nav_control_cmd";

  // ── 发布者：aim_target (/aim_target → PointStamped) ──
  BT::RosNodeParams params_aim_target;
  params_aim_target.nh = std::make_shared<rclcpp::Node>("rmuc_aim_target_io");
  params_aim_target.default_port_value = "/aim_target";

  // ── 通用 ROS 节点（不绑定特定消息话题，供工具类插件使用） ──
  BT::RosNodeParams params_utility;
  params_utility.nh = std::make_shared<rclcpp::Node>("rmuc_utility");
  params_utility.default_port_value = "";

  // ── SendGoal (共享 RMUL 通用导航话题，PoseStamped 类型) ──
  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
  params_send_goal.default_port_value = "goal_pose";

  // ═══════════════════ 注册插件 ══════════════════════════════════
  // 辅助 lambda：注册 ROS 节点插件
  auto regRos = [&](const std::string & lib, const BT::RosNodeParams & p) {
    try {
      RegisterRosNode(factory, BT::SharedLibrary::getOSName(lib), p);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load ROS plugin '%s': %s",
                  lib.c_str(), e.what());
    }
  };
  // 辅助 lambda：注册纯 BT 插件
  auto regBT = [&](const std::string & lib) {
    try {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(lib));
    } catch (const std::exception & e) {
      RCLCPP_WARN(node->get_logger(), "Could not load BT plugin '%s': %s",
                  lib.c_str(), e.what());
    }
  };

  // clang-format off

  // ── A. 订阅者：game_status (/game_status → RMUCGameStatus) ──
  regRos("rmuc_sub_game_status",                params_game_status);

  // ── B. 订阅者：robot_status (/robot_status → RMUCRobotStatus) ──
  regRos("rmuc_sub_robot_status",               params_robot_status);
  regRos("rmuc_detect_respawn_and_set_recovery", params_robot_status);
  regRos("rmuc_wait_and_heal",                  params_robot_status);

  // ── C. 订阅者：rfid_status (/rfid_status → RMUCRFIDStatus) ──
  regRos("rmuc_sub_rfid_status",                params_rfid_status);

  // ── D. 订阅者：robot_position (/robot_position → RMUCRobotPosition) ──
  regRos("rmuc_sub_robot_position",             params_robot_position);

  // ── E. 订阅者：radar/enemy_tracks (/radar/enemy_tracks → RMUCEnemyTracks) ──
  regRos("rmuc_sub_radar_tracks",               params_radar);

  // ── F. 发布者：sentry_cmd (/sentry_cmd → RMUCSentryCmd) ──
  regRos("rmuc_sentry_cmd_mux",                 params_sentry_cmd);

  // ── G. 发布者：robot_control (/robot_control → RMUCRobotControl) ──
  regRos("rmuc_robot_control",                  params_robot_ctrl);

  // ── H. 发布者：nav_control_cmd (/nav_control_cmd → RMUCNavControlCmd) ──
  regRos("rmuc_nav_control_cmd",                params_nav_cmd);

  // ── I. 工具类 ROS 插件（不绑定特定话题，仅需 ROS node handle） ──
  regRos("rmuc_micro_search_supply_card",       params_utility);
  regRos("rmuc_is_supply_card_detected",        params_utility);

  // ── J. 共享 RMUL ROS 插件 ──
  regRos("cancel_nav_goal",                     params_utility);
  regRos("clear_recovery_flag",                 params_utility);
  regRos("init_search_timer_if_needed",         params_utility);
  regRos("is_recovery_needed",                  params_utility);

  // ── K. SendGoal (PoseStamped，非 RMUC 消息) ──
  regRos("send_goal",                           params_send_goal);

  // ── L. RMUC 纯 BT 插件（无需 ROS 参数，通过黑板获取数据） ──
  // 动作
  regBT("rmuc_init_sentry_config");
  regBT("rmuc_init_cmd_state");
  regBT("rmuc_decide_posture");
  regBT("rmuc_decide_economy_cmd");
  regBT("rmuc_decide_respawn_cmd");
  regBT("rmuc_parse_sentry_blackboard");
  regBT("rmuc_select_safe_retreat_goal");
  regBT("rmuc_select_best_target");
  regRos("rmuc_aim_at_target",                  params_aim_target);
  regBT("rmuc_fire_burst");
  regBT("rmuc_hold_and_heal");
  regBT("rmuc_hold_for_supply_ammo_tick");
  regBT("rmuc_select_nearest_resupply_station");
  regBT("rmuc_select_nearest_dispel_card");
  regBT("rmuc_select_objective");
  regBT("rmuc_hold_objective");
  regBT("rmuc_waypoint_patrol");
  // 条件
  regBT("rmuc_is_dead");
  regBT("rmuc_is_game_time");
  regBT("rmuc_is_hp_below");
  regBT("rmuc_is_at_nav_goal");
  regBT("rmuc_is_at_goal");
  regBT("rmuc_is_zone_card_detected");
  regBT("rmuc_is_any_dispel_card_detected");
  regBT("rmuc_is_critical_state");
  regBT("rmuc_is_base_threatened");
  regBT("rmuc_has_valid_target");
  regBT("rmuc_is_combat_allowed");
  regBT("rmuc_is_fire_window_ok");
  regBT("rmuc_is_ammo_below");

  // ── M. 共享 RMUL BT 插件 ──
  regBT("rate_controller");
  regBT("keep_running");
  regBT("move_around");

  // clang-format on

  // ═══════════════════ 创建并执行行为树 ═════════════════════════

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create behavior tree from '%s': %s",
                 bt_xml_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Groot2Publisher 端口 1668（RMUL 用 1667，避免冲突）
  const unsigned port = 1668;
  BT::Groot2Publisher publisher(tree, port);

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
