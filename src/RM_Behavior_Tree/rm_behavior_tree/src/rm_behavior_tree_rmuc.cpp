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
 *   /robot_position    — RMUCRobotPosition   (50 Hz)
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

#include <string>
#include <utility>
#include <vector>

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
  params_game_status.default_port_value = "game_status";

  BT::RosNodeParams params_robot_status;
  params_robot_status.nh = std::make_shared<rclcpp::Node>("rmuc_robot_status_io");
  params_robot_status.default_port_value = "robot_status";

  BT::RosNodeParams params_robot_buff;
  params_robot_buff.nh = std::make_shared<rclcpp::Node>("rmuc_robot_buff_io");
  params_robot_buff.default_port_value = "robot_buff";

  BT::RosNodeParams params_robot_position;
  params_robot_position.nh = std::make_shared<rclcpp::Node>("rmuc_robot_position_io");
  params_robot_position.default_port_value = "robot_position";

  BT::RosNodeParams params_radar_tracks;
  params_radar_tracks.nh = std::make_shared<rclcpp::Node>("rmuc_radar_tracks_io");
  params_radar_tracks.default_port_value = "radar/enemy_tracks";

  // ── 输出话题（发布者） ──
  BT::RosNodeParams params_sentry_cmd;
  params_sentry_cmd.nh = std::make_shared<rclcpp::Node>("rmuc_sentry_cmd_io");
  params_sentry_cmd.default_port_value = "sentry_cmd";

  BT::RosNodeParams params_robot_ctrl;
  params_robot_ctrl.nh = std::make_shared<rclcpp::Node>("rmuc_robot_ctrl_io");
  params_robot_ctrl.default_port_value = "robot_control";

  BT::RosNodeParams params_nav_cmd;
  params_nav_cmd.nh = std::make_shared<rclcpp::Node>("rmuc_nav_cmd_io");
  params_nav_cmd.default_port_value = "nav_control_cmd";

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

  // ── B2. 订阅者：robot_buff (/robot_buff → RMUCRobotBuff) ──
  regRos("rmuc_sub_robot_buff",                 params_robot_buff);

  // ── C. 订阅者：robot_position (/robot_position → RMUCRobotPosition) ──
  regRos("rmuc_sub_robot_position",             params_robot_position);

  // ── D2. 订阅者：radar_tracks (/radar/enemy_tracks → RMUCEnemyTracks) ──
  regRos("rmuc_sub_radar_tracks",               params_radar_tracks);

  // ── F. 发布者：sentry_cmd (/sentry_cmd → RMUCSentryCmd) ──
  regRos("rmuc_sentry_cmd_mux",                 params_sentry_cmd);

  // ── G. 发布者：robot_control (/robot_control → RMUCRobotControl) ──
  regRos("rmuc_robot_control",                  params_robot_ctrl);

  // ── H. 发布者：nav_control_cmd (/nav_control_cmd → RMUCNavControlCmd) ──
  regRos("rmuc_nav_control_cmd",                params_nav_cmd);

  // ── J. 共享 RMUL ROS 插件 ──
  regRos("cancel_nav_goal",                     params_utility);
  regRos("clear_recovery_flag",                 params_utility);
  regRos("is_recovery_needed",                  params_utility);

  // ── J2. Nav2 导航目标查询 ──
  regRos("rmuc_is_nav_target_supply",           params_utility);

  // ── K. SendGoal (PoseStamped，非 RMUC 消息) ──
  regRos("send_goal",                           params_send_goal);

  // ── K2. IsAtGoal (需要订阅 global_costmap/costmap 做视线检查) ──
  BT::RosNodeParams params_costmap;
  params_costmap.nh = std::make_shared<rclcpp::Node>("is_at_goal_costmap_io");
  params_costmap.default_port_value = "global_costmap/costmap";
  regRos("rmuc_is_at_goal",                     params_costmap);

  // ── L. RMUC 纯 BT 插件（无需 ROS 参数，通过黑板获取数据） ──
  // 动作
  regBT("rmuc_init_sentry_config");
  regBT("rmuc_init_cmd_state");
  regBT("rmuc_update_heal_recovery_latch");
  regBT("rmuc_load_calibration_csv");
  regBT("rmuc_select_posture");
  regBT("rmuc_select_sentry_cmd_rate");
  regBT("rmuc_posture_degradation_guard");
  regBT("rmuc_decide_respawn_cmd");
  regBT("rmuc_parse_sentry_blackboard");
  regBT("rmuc_hold_and_heal");
  regBT("rmuc_hold_for_supply_ammo_tick");
  regBT("rmuc_select_nearest_resupply_station");
  regBT("rmuc_select_objective");
  regBT("rmuc_hold_objective");
  regBT("rmuc_objective_patrol");
  // 条件
  regBT("rmuc_is_dead");
  regBT("rmuc_is_game_time");
  regBT("rmuc_is_hp_below");
  // rmuc_is_at_goal 已移至 K2（需要 costmap 订阅）
  regBT("rmuc_is_zone_card_detected");
  regBT("rmuc_is_base_threatened");
  regBT("rmuc_is_vulnerable");
  regBT("rmuc_is_detect_enemy");
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

  // ─── RMUC 2026 参数注入 ───
  // 从 ROS 参数 (rmuc_sentry_config.*) 读取，注入到树的根黑板
  // InitSentryConfig 的 getInput 会优先读取黑板中已有的值
  {
    auto bb = tree.rootBlackboard();
    const std::vector<std::string> coord_keys = {
      "home_x","home_y","supply_zone_x","supply_zone_y",
      "base_x","base_y",
      "base_buff_x","base_buff_y","outpost_buff_x","outpost_buff_y",
      "central_highland_x","central_highland_y",
      "ladder_highland_x","ladder_highland_y",
      "defend_anchor_x","defend_anchor_y",
      "central_highland_left_x","central_highland_left_y",
      "ramp_jump_x","ramp_jump_y",
      "cap_outpost_x","cap_outpost_y"
    };
    const std::vector<std::pair<std::string, double>> double_keys = {
      {"arrive_radius", 1.0},
      {"enemy_near_base_radius", 0.0},
      {"move_around_expected_dis", 0.3}
    };
    const std::vector<std::pair<std::string, int>> int_keys = {
      {"hp_low", 180}, {"hp_safe", 280},
      {"ammo_low", 80},
      {"base_threat_calm_timeout_ms", 0},
      {"cap_sustain_time", 30},
      {"patrol_hold_ms", 5000},
      {"move_around_expected_nearby_goal_count", 3}
    };

    const std::string prefix = "rmuc_sentry_config.";
    int injected = 0;

    for (const auto & k : coord_keys) {
      auto param_name = prefix + k;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<double>(param_name, 0.0);
      }
      double v = node->get_parameter(param_name).as_double();
      bb->set("cfg." + k, v);
      injected++;
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
      if (k == "cap_sustain_time") {
        bb->set(k, v);
      }
      injected++;
    }
    RCLCPP_INFO(node->get_logger(), "Injected %d RMUC config params into blackboard", injected);

    // patrol_enable (bool)
    {
      auto pn = prefix + "patrol_enable";
      if (!node->has_parameter(pn)) node->declare_parameter<bool>(pn, false);
      bool v = node->get_parameter(pn).as_bool();
      bb->set("cfg.patrol_enable", v);
      injected++;
    }
    // patrol_waypoints (vector<double> → string "x1,y1;x2,y2;...")
    {
      auto pn = prefix + "patrol_waypoints";
      if (!node->has_parameter(pn))
        node->declare_parameter<std::vector<double>>(pn, std::vector<double>{});
      auto vec = node->get_parameter(pn).as_double_array();
      std::string wpts_str;
      for (size_t i = 0; i + 1 < vec.size(); i += 2) {
        if (!wpts_str.empty()) wpts_str += ";";
        wpts_str += std::to_string(vec[i]) + "," + std::to_string(vec[i + 1]);
      }
      bb->set("cfg.patrol_waypoints", wpts_str);
      if (!wpts_str.empty()) injected++;
      RCLCPP_INFO(node->get_logger(), "Patrol waypoints (%zu points): %s",
                  vec.size() / 2, wpts_str.c_str());
    }
    // calibration_csv_path (string)
    for (const auto & [key, def] : std::vector<std::pair<std::string, std::string>>{
        {"calibration_csv_path", ""},
        {"semantic_zones_file",
          "/ws/src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/semantic_zones.yaml"},
        {"semantic_ignore_enemy_zone_type", "speed_bump"}})
    {
      auto pn = prefix + key;
      if (!node->has_parameter(pn)) node->declare_parameter<std::string>(pn, def);
      std::string v = node->get_parameter(pn).as_string();
      bb->set("cfg." + key, v);
      if (!v.empty()) {
        RCLCPP_INFO(node->get_logger(), "%s: %s", key.c_str(), v.c_str());
      }
    }
  }

  // Groot2Publisher 端口 1668（RMUL 用 1667，避免冲突）
  std::unique_ptr<BT::Groot2Publisher> publisher;
  try {
    const unsigned port = 2668;
    publisher = std::make_unique<BT::Groot2Publisher>(tree, port);
    RCLCPP_INFO(node->get_logger(), "Groot2Publisher started on port %u", port);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "Failed to start Groot2Publisher on port 2668: %s. Continuing without it.", e.what());
  }

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
