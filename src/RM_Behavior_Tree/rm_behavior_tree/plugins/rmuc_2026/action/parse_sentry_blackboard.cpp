#include "rm_behavior_tree/plugins/rmuc_2026/action/parse_sentry_blackboard.hpp"
#include <iostream>

#include "yaml-cpp/yaml.h"

namespace rm_behavior_tree
{

namespace
{
constexpr double kDefaultBaseX = -2.3532;
constexpr double kDefaultBaseY = -2.0007;
constexpr double kDefaultBaseThreatEnterDistance = 5.0;
constexpr int kDefaultBaseThreatCalmTimeoutMs = 30000;
constexpr char kDefaultSemanticZonesFile[] = "/ws/src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/semantic_zones.yaml";
}

ParseSentryBlackboardAction::ParseSentryBlackboardAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

void ParseSentryBlackboardAction::resetSemanticZonesState()
{
  semantic_zones_.clear();
  semantic_zones_loaded_ = false;
  semantic_zones_load_attempted_ = false;
  semantic_zones_file_.clear();
}

void ParseSentryBlackboardAction::logSemanticZonesLoadErrorOnce(
  const std::string & yaml_path, const std::string & error)
{
  const std::string formatted_error = yaml_path + "|" + error;
  if (semantic_zones_last_error_ == formatted_error) {
    return;
  }

  semantic_zones_last_error_ = formatted_error;
  std::cerr << "[ParseSentryBlackboard] semantic zones load failed: "
            << yaml_path << " (" << error << ")" << std::endl;
}

void ParseSentryBlackboardAction::loadSemanticZones(const std::string & yaml_path)
{
  semantic_zones_.clear();
  semantic_zones_loaded_ = false;
  semantic_zones_load_attempted_ = true;
  semantic_zones_file_ = yaml_path;

  if (yaml_path.empty()) {
    logSemanticZonesLoadErrorOnce(yaml_path, "semantic_zones_file resolved to an empty path");
    return;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    logSemanticZonesLoadErrorOnce(yaml_path, e.what());
    return;
  }

  if (!config["zones"]) {
    logSemanticZonesLoadErrorOnce(yaml_path, "missing 'zones' field");
    return;
  }

  for (const auto & z : config["zones"]) {
    if (!z["name"] || !z["type"] || !z["vertices"]) {
      continue;
    }

    RmucSemanticZone zone;
    zone.name = z["name"].as<std::string>();
    zone.type = z["type"].as<std::string>();
    for (const auto & v : z["vertices"]) {
      if (v.size() < 2) {
        continue;
      }
      zone.vertices.emplace_back(v[0].as<double>(), v[1].as<double>());
    }

    if (zone.vertices.size() >= 3) {
      semantic_zones_.push_back(std::move(zone));
    }
  }

  semantic_zones_loaded_ = true;
}

bool ParseSentryBlackboardAction::pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & poly) const
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i].first;
    const double yi = poly[i].second;
    const double xj = poly[j].first;
    const double yj = poly[j].second;
    if (((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

bool ParseSentryBlackboardAction::isInSemanticZone(double x, double y, const std::string & zone_type)
{
  for (const auto & zone : semantic_zones_) {
    if (zone.type == zone_type && pointInPolygon(x, y, zone.vertices)) {
      return true;
    }
  }
  return false;
}

BT::NodeStatus ParseSentryBlackboardAction::tick()
{
  // ── 比赛阶段 (GameStatus) ──
  auto game_msg = getInput<sp_msgs::msg::RMUCGameStatus>("game_status");
  if (game_msg) {
    setOutput("stage_remain_time", static_cast<int>(game_msg->stage_remain_time));
    setOutput("stage_elapsed_time", 420 - static_cast<int>(game_msg->stage_remain_time));
  }

  bool suppress_enemy_detection = false;
  double pose_x = 0.0;
  double pose_y = 0.0;
  std::string semantic_zones_file = kDefaultSemanticZonesFile;
  std::string semantic_ignore_zone_type = "speed_bump";
  const bool has_pose = getInput("pose_x", pose_x) && getInput("pose_y", pose_y);
  const auto semantic_zones_file_result = getInput("semantic_zones_file", semantic_zones_file);
  getInput("semantic_ignore_enemy_zone_type", semantic_ignore_zone_type);
  if (!semantic_zones_file_result.has_value()) {
    logSemanticZonesLoadErrorOnce(
      kDefaultSemanticZonesFile,
      std::string("cfg.semantic_zones_file read failed: ") + semantic_zones_file_result.error() +
      ", fallback to default path");
    semantic_zones_file = kDefaultSemanticZonesFile;
  } else if (semantic_zones_file.empty()) {
    logSemanticZonesLoadErrorOnce(
      kDefaultSemanticZonesFile,
      "cfg.semantic_zones_file is empty, fallback to default path");
    semantic_zones_file = kDefaultSemanticZonesFile;
  }

  if (has_pose) {
    if (!semantic_zones_load_attempted_ || semantic_zones_file_ != semantic_zones_file) {
      loadSemanticZones(semantic_zones_file);
    }
    suppress_enemy_detection = semantic_zones_loaded_ &&
      isInSemanticZone(pose_x, pose_y, semantic_ignore_zone_type);
  }
  if (suppress_enemy_detection != semantic_enemy_override_active_) {
    semantic_enemy_override_active_ = suppress_enemy_detection;
    std::cout << "[ParseSentryBlackboard] semantic enemy override "
              << (semantic_enemy_override_active_ ? "enabled" : "disabled")
              << " (zone_type=" << semantic_ignore_zone_type << ")"
              << std::endl;
  }

  bool effective_detect_enemy = false;

  // ── 机器人状态 (RobotStatus) ──
  auto robot_ptr = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status");
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    effective_detect_enemy = suppress_enemy_detection ? false : r.is_detect_enemy;
    setOutput("hp_cur", static_cast<int>(r.current_hp));
    setOutput("ammo_allow", static_cast<int>(r.ammo_allow));
    // base_hp_cur: 已改由 team_hp.base_hp 接管 (0x0003 offset 14)
    // outpost_alive 不再使用 robot_status.outpost_alive (bool转换可能有误)
    // 将由 team_hp.outpost_hp > 0 得出（0x0003 offset 12 原始 uint16_t)
    setOutput("is_dead", r.current_hp <= 0);
    setOutput("has_target", effective_detect_enemy);
    setOutput("is_detect_enemy", effective_detect_enemy);
  } else {
    setOutput("has_target", false);
    setOutput("is_detect_enemy", false);
  }

  auto radar_tracks = getInput<sp_msgs::msg::RMUCEnemyTracks>("radar_tracks");
  // 注意: 雷达仅用于基地威胁判断，不参与 has_target (战斗仅依赖云台视觉 is_detect_enemy)

  // ── 队伍血量 (TeamHP) ──
  auto th = getInput<sp_msgs::msg::RMUCTeamHP>("team_hp");
  if (th) {
    // 引用原始 uint16_t 字段 (0x0003 offset 12/14)，避免 robot_status bool转换失真
    setOutput("outpost_alive", th->outpost_hp > 0);
    setOutput("base_hp_cur", static_cast<int>(th->base_hp));  // 0x0003 offset 14 接管
  }

  // 基地危机锁存：
  // 进入条件：雷达扫描到敌人在基地坐标附近 enemy_near_base_radius 内 + 基地 HP 下降
  // 退出条件：云台没有扫描到敌人 + 基地不掉血，持续 base_threat_calm_timeout_ms 自动解除
  bool base_threat = base_threat_latched_;
  double base_x = kDefaultBaseX;
  double base_y = kDefaultBaseY;
  double base_threat_enter_distance = kDefaultBaseThreatEnterDistance;
  int base_threat_calm_timeout_ms = kDefaultBaseThreatCalmTimeoutMs;
  const auto base_x_result = getInput("base_x", base_x);
  const auto base_y_result = getInput("base_y", base_y);
  const auto base_radius_result = getInput("enemy_near_base_radius", base_threat_enter_distance);
  const auto base_calm_timeout_result = getInput(
    "base_threat_calm_timeout_ms", base_threat_calm_timeout_ms);
  const bool has_base_x = base_x_result.has_value();
  const bool has_base_y = base_y_result.has_value();
  const bool has_base_radius = base_radius_result.has_value();
  const bool has_base_calm_timeout = base_calm_timeout_result.has_value();

  if ((!has_base_x || !has_base_y) && !logged_missing_base_config_) {
    std::cerr << "[ParseSentryBlackboard] 因为 cfg.base_x/base_y 读取失败"
              << " (base_x=" << (has_base_x ? "ok" : base_x_result.error())
              << ", base_y=" << (has_base_y ? "ok" : base_y_result.error())
              << ")，导致无法从配置链获取基地坐标，现采用默认值 base=("
              << kDefaultBaseX << ", " << kDefaultBaseY << ")"
              << std::endl;
    logged_missing_base_config_ = true;
  }
  if (has_base_x && has_base_y) {
    logged_missing_base_config_ = false;
  }

  if (!has_base_radius && !logged_missing_base_radius_) {
    std::cerr << "[ParseSentryBlackboard] 因为 cfg.enemy_near_base_radius 读取失败 ("
              << base_radius_result.error()
              << ")，导致无法从配置链获取基地威胁进入半径，现采用默认值 "
              << kDefaultBaseThreatEnterDistance << "m"
              << std::endl;
    logged_missing_base_radius_ = true;
  }
  if (has_base_radius) {
    logged_missing_base_radius_ = false;
  }

  if (!has_base_calm_timeout && !logged_missing_base_calm_timeout_) {
    std::cerr << "[ParseSentryBlackboard] 因为 cfg.base_threat_calm_timeout_ms 读取失败 ("
              << base_calm_timeout_result.error()
              << ")，导致无法从配置链获取基地威胁解除平静时间，现采用默认值 "
              << kDefaultBaseThreatCalmTimeoutMs << "ms"
              << std::endl;
    logged_missing_base_calm_timeout_ = true;
  }
  if (has_base_calm_timeout) {
    logged_missing_base_calm_timeout_ = false;
  }

  // 雷达距离计算（用于进入条件）
  bool any_enemy_near = false;
  if (radar_tracks &&
    radar_tracks->enemy_x.size() == radar_tracks->enemy_y.size() &&
    !radar_tracks->enemy_x.empty())
  {
    for (size_t index = 0; index < radar_tracks->enemy_x.size(); ++index) {
      const double dx = static_cast<double>(radar_tracks->enemy_x[index]) - base_x;
      const double dy = static_cast<double>(radar_tracks->enemy_y[index]) - base_y;
      const double distance = std::hypot(dx, dy);
      if (distance < base_threat_enter_distance) {
        any_enemy_near = true;
        break;
      }
    }
  }

  // 进入条件：雷达扫到敌人在基地附近 + 基地掉血
  bool base_hp_is_dropping = false;
  auto now = std::chrono::steady_clock::now();
  if (th) {
    base_hp_is_dropping = (last_base_hp_ >= 0 && th->base_hp < static_cast<uint16_t>(last_base_hp_));
    if (base_hp_is_dropping && any_enemy_near) {
      base_threat = true;
      const bool should_log_trigger = !base_threat_latched_ ||
        !has_base_threat_trigger_log_ ||
        (now - last_base_threat_trigger_log_) >= std::chrono::seconds(5);
      if (should_log_trigger) {
        std::cout << "[ParseSentryBlackboard] 基地威胁触发：雷达扫到敌人在基地 "
                  << base_threat_enter_distance << "m 内且基地掉血"
                  << std::endl;
        last_base_threat_trigger_log_ = now;
        has_base_threat_trigger_log_ = true;
      }
    }
    last_base_hp_ = static_cast<int>(th->base_hp);
  }

  // 退出条件：云台没扫到敌人 + 基地不掉血，持续 base_threat_calm_timeout_ms 自动解除
  const bool gimbal_detects_enemy = effective_detect_enemy;

  if (base_threat) {
    bool is_calm = !gimbal_detects_enemy && !base_hp_is_dropping;
    if (is_calm) {
      if (!base_threat_calm_tracking_) {
        base_threat_calm_tracking_ = true;
        base_threat_calm_start_ = now;
      } else {
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - base_threat_calm_start_).count();
        if (has_base_calm_timeout && base_threat_calm_timeout_ms > 0 &&
          elapsed_ms >= base_threat_calm_timeout_ms)
        {
          base_threat = false;
          base_threat_calm_tracking_ = false;
          std::cout << "[ParseSentryBlackboard] 基地威胁解除：连续 "
                    << base_threat_calm_timeout_ms
                    << "ms 云台未检测到敌人且基地不掉血"
                    << std::endl;
        }
      }
    } else {
      // 云台检测到敌人或基地在掉血 → 重置平静计时器
      base_threat_calm_tracking_ = false;
    }
  } else {
    base_threat_calm_tracking_ = false;
  }

  base_threat_latched_ = base_threat;
  setOutput("base_threat", base_threat);

  // ── 脱战检测 ──
  // 规则: 存活状态下连续 6 秒未发射弹丸且未被扣血 = 脱战。
  // 开局默认脱战。
  std::uint64_t now_ms = 0;
  getInput("now_ms", now_ms);

  if (robot_ptr) {
    const auto & rd = **robot_ptr;
    if (!disengage_initialized_) {
      last_shooter_heat_ = rd.shooter_heat;
      last_current_hp_ = rd.current_hp;
      last_activity_ms_ = 0;  // 开局视为脱战(activity=0 → elapsed > 6s 立即成立)
      disengage_initialized_ = true;
    }

    const bool fired = (rd.shooter_heat > last_shooter_heat_);
    const bool took_damage = (rd.current_hp < last_current_hp_ && last_current_hp_ > 0);

    if (fired || took_damage || rd.current_hp <= 0) {
      last_activity_ms_ = now_ms;
    }
    last_shooter_heat_ = rd.shooter_heat;
    last_current_hp_ = rd.current_hp;

    const bool is_disengaged = (rd.current_hp > 0 && now_ms > 0 &&
      (now_ms - last_activity_ms_) >= 6000);
    setOutput("is_disengaged", is_disengaged);
  } else {
    setOutput("is_disengaged", false);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
