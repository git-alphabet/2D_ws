#include "rm_behavior_tree/plugins/rmul_2026/condition/is_in_semantic_zone.hpp"

#include <string>
#include <vector>

#include "behaviortree_ros2/plugins.hpp"
#include "yaml-cpp/yaml.h"

namespace rm_behavior_tree
{

IsInSemanticZoneCondition::IsInSemanticZoneCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf)
{
  (void)params;
}

BT::NodeStatus IsInSemanticZoneCondition::tick()
{
  auto res_x = getInput<double>("pose_x");
  auto res_y = getInput<double>("pose_y");
  if (!res_x || !res_y) {
    return BT::NodeStatus::FAILURE;
  }

  std::string zone_type = "keepout";
  getInput("zone_type", zone_type);

  std::string zones_file;
  getInput("zones_file", zones_file);

  // Lazy load: load yaml once (or when file path changes)
  if (!loaded_ || loaded_file_ != zones_file) {
    loadZones(zones_file);
  }

  const double px = res_x.value();
  const double py = res_y.value();

  for (const auto & zone : zones_) {
    if (zone.type == zone_type && pointInPolygon(px, py, zone.vertices)) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

void IsInSemanticZoneCondition::loadZones(const std::string & yaml_path)
{
  zones_.clear();
  loaded_ = false;
  loaded_file_ = yaml_path;

  if (yaml_path.empty()) {
    return;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &) {
    return;
  }

  if (!config["zones"]) {
    return;
  }

  for (const auto & z : config["zones"]) {
    SemanticZoneDef zone;
    zone.name = z["name"].as<std::string>();
    zone.type = z["type"].as<std::string>();

    for (const auto & v : z["vertices"]) {
      zone.vertices.emplace_back(v[0].as<double>(), v[1].as<double>());
    }

    if (zone.vertices.size() >= 3) {
      zones_.push_back(std::move(zone));
    }
  }

  loaded_ = true;
}

bool IsInSemanticZoneCondition::pointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>> & poly) const
{
  bool inside = false;
  size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    double xi = poly[i].first, yi = poly[i].second;
    double xj = poly[j].first, yj = poly[j].second;
    if (((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::IsInSemanticZoneCondition, "IsInSemanticZone");
