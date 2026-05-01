#include "rm_behavior_tree/plugins/rmuc_2026/action/load_calibration_csv.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

namespace rm_behavior_tree
{

LoadCalibrationCSVAction::LoadCalibrationCSVAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus LoadCalibrationCSVAction::tick()
{
  std::string csv_path;
  getInput("csv_path", csv_path);

  if (csv_path.empty()) {
    if (!logged_empty_path_) {
      std::cout << "[CALIB_CSV] csv_path 为空，跳过标定覆盖，使用 YAML 默认值\n";
      logged_empty_path_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (loaded_once_) {
    // 检查文件是否被修改（标定工具可能在运行期间更新 CSV）
    namespace fs = std::filesystem;
    std::error_code ec;
    auto mtime = fs::last_write_time(csv_path, ec);
    if (ec || mtime == last_mtime_) {
      return BT::NodeStatus::SUCCESS;
    }
    std::cout << "[CALIB_CSV] 检测到标定文件变更，重新加载...\n";
    // fall through to re-load
  }

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    if (!logged_open_fail_) {
      std::cerr << "[CALIB_CSV] 无法打开标定文件: " << csv_path
                << "，使用 YAML 默认值\n";
      logged_open_fail_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 已知坐标点名称 (不含 _x/_y 后缀)
  static const std::vector<std::string> known_points = {
    "home", "supply_zone", "base", "base_buff", "outpost_buff",
    "central_highland",
    "ladder_highland", "defend_anchor",
    "central_highland_left", "ramp_jump"
  };

  // 辅助: 去除首尾空白
  auto trim = [](std::string & s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) { s.clear(); return; }
    s = s.substr(start, s.find_last_not_of(" \t\r\n") - start + 1);
  };

  // 解析 CSV
  std::map<std::string, std::pair<double, double>> point_map;
  std::map<int, std::pair<double, double>> patrol_map;

  std::string line;
  int line_num = 0;
  while (std::getline(file, line)) {
    line_num++;
    trim(line);
    if (line.empty() || line[0] == '#') continue;

    std::istringstream ss(line);
    std::string name, x_str, y_str;

    if (!std::getline(ss, name, ',')) continue;
    if (!std::getline(ss, x_str, ',')) continue;
    if (!std::getline(ss, y_str)) continue;

    trim(name);
    trim(x_str);
    trim(y_str);

    double x, y;
    try {
      x = std::stod(x_str);
      y = std::stod(y_str);
    } catch (...) {
      std::cerr << "[CALIB_CSV] 第 " << line_num << " 行坐标解析失败: " << line << "\n";
      continue;
    }

    // 巡逻点 (patrol_1, patrol_2, ...)
    if (name.size() > 7 && name.substr(0, 7) == "patrol_") {
      try {
        int idx = std::stoi(name.substr(7));
        patrol_map[idx] = {x, y};
      } catch (...) {
        std::cerr << "[CALIB_CSV] 无法解析巡逻点序号: " << name << "\n";
      }
      continue;
    }

    // 普通坐标点
    point_map[name] = {x, y};
  }
  file.close();

  // 覆写已识别的坐标点 (跳过 (0,0) — 视为未标定)
  int overridden = 0;
  for (const auto & pt_name : known_points) {
    auto it = point_map.find(pt_name);
    if (it == point_map.end()) continue;

    const auto & [x, y] = it->second;
    // (0,0) 视为未标定，保留 YAML 默认值
    if (std::abs(x) < 1e-6 && std::abs(y) < 1e-6) {
      std::cout << "[CALIB_CSV] 跳过 " << pt_name << " (0,0) → 使用 YAML 默认值\n";
      continue;
    }
    setOutput(pt_name + "_x", x);
    setOutput(pt_name + "_y", y);
    overridden++;
    std::cout << "[CALIB_CSV] 覆盖 " << pt_name << " → (" << x << ", " << y << ")\n";
  }

  // 覆写巡逻航点 (重建 "x1,y1;x2,y2;..." 格式字符串)
  if (!patrol_map.empty()) {
    std::string wpts_str;
    for (const auto & [idx, coord] : patrol_map) {
      if (!wpts_str.empty()) wpts_str += ";";
      wpts_str += std::to_string(coord.first) + "," + std::to_string(coord.second);
    }
    setOutput("patrol_waypoints", wpts_str);
    overridden++;
    std::cout << "[CALIB_CSV] 覆盖 patrol_waypoints (" << patrol_map.size()
              << " 点): " << wpts_str << "\n";
  }

  // 警告未识别的点位
  for (const auto & [name, _] : point_map) {
    bool found = false;
    for (const auto & kp : known_points) {
      if (kp == name) { found = true; break; }
    }
    if (!found) {
      std::cerr << "[CALIB_CSV] 未识别的点位名称 (已忽略): " << name << "\n";
    }
  }

  std::cout << "[CALIB_CSV] 标定覆盖完成: " << overridden << " 项来自 " << csv_path << "\n";
  loaded_once_ = true;

  // 记录文件修改时间，用于检测后续变更
  {
    namespace fs = std::filesystem;
    std::error_code ec;
    last_mtime_ = fs::last_write_time(csv_path, ec);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::LoadCalibrationCSVAction>("LoadCalibrationCSV");
}
