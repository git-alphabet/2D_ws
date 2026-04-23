// Copyright 2026 GXU RMUL Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pb_nav2_plugins/layers/semantic_zone_layer.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "yaml-cpp/yaml.h"

namespace pb_nav2_costmap_2d
{

namespace
{
constexpr unsigned char SPEED_BUMP_COST = 180;
}

void SemanticZoneLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SemanticZoneLayer: failed to lock node");
  }

  // Declare enabled parameter and read it
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  // Mark this layer as always current (static data, no sensor dependency)
  current_ = true;

  // Declare and read the yaml path parameter
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".zones_file", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".target_zone_name", rclcpp::ParameterValue(target_zone_name_));

  std::string zones_file;
  node->get_parameter(name_ + ".zones_file", zones_file);
  node->get_parameter(name_ + ".target_zone_name", target_zone_name_);

  if (zones_file.empty()) {
    RCLCPP_WARN(node->get_logger(), "SemanticZoneLayer: zones_file is empty, no zones loaded");
    return;
  }

  loadZonesFromYaml(zones_file);
}

void SemanticZoneLayer::loadZonesFromYaml(const std::string & yaml_path)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  zones_.clear();
  zones_loaded_ = false;

  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "SemanticZoneLayer: failed to load %s: %s",
      yaml_path.c_str(), e.what());
    return;
  }

  if (!config["zones"]) {
    RCLCPP_WARN(node->get_logger(), "SemanticZoneLayer: no 'zones' key in %s", yaml_path.c_str());
    return;
  }

  zone_min_x_ = std::numeric_limits<double>::max();
  zone_min_y_ = std::numeric_limits<double>::max();
  zone_max_x_ = std::numeric_limits<double>::lowest();
  zone_max_y_ = std::numeric_limits<double>::lowest();

  for (const auto & z : config["zones"]) {
    SemanticZone zone;
    if (!z["name"] || !z["vertices"]) {
      RCLCPP_WARN(node->get_logger(), "SemanticZoneLayer: malformed zone entry in %s", yaml_path.c_str());
      continue;
    }

    zone.name = z["name"].as<std::string>();
    zone.type = z["type"] ? z["type"].as<std::string>() : "";
    if (zone.name != target_zone_name_) {
      continue;
    }
    if (!zone.type.empty() && zone.type != "speed_bump") {
      RCLCPP_WARN(
        node->get_logger(),
        "SemanticZoneLayer: zone '%s' type '%s' is unsupported, expected speed_bump",
        zone.name.c_str(), zone.type.c_str());
      continue;
    }
    if (zone.type.empty()) {
      zone.type = "speed_bump";
    }
    if (!zones_.empty()) {
      RCLCPP_WARN(
        node->get_logger(),
        "SemanticZoneLayer: multiple '%s' zones found, only the first valid zone is kept",
        target_zone_name_.c_str());
      continue;
    }

    // Parse vertices
    for (const auto & v : z["vertices"]) {
      if (!v.IsSequence() || v.size() < 2) {
        continue;
      }
      double vx = v[0].as<double>();
      double vy = v[1].as<double>();
      zone.vertices.emplace_back(vx, vy);

      zone_min_x_ = std::min(zone_min_x_, vx);
      zone_min_y_ = std::min(zone_min_y_, vy);
      zone_max_x_ = std::max(zone_max_x_, vx);
      zone_max_y_ = std::max(zone_max_y_, vy);
    }

    if (zone.vertices.size() >= 3) {
      RCLCPP_INFO(
        node->get_logger(), "SemanticZoneLayer: loaded zone '%s' (type=%s, vertices=%zu, cost=%u)",
        zone.name.c_str(), zone.type.c_str(), zone.vertices.size(), SPEED_BUMP_COST);
      zones_.push_back(std::move(zone));
    }
  }

  zones_loaded_ = !zones_.empty();
  if (!zones_loaded_) {
    zone_min_x_ = 0.0;
    zone_min_y_ = 0.0;
    zone_max_x_ = 0.0;
    zone_max_y_ = 0.0;
    RCLCPP_WARN(
      node->get_logger(),
      "SemanticZoneLayer: target zone '%s' not found in %s",
      target_zone_name_.c_str(), yaml_path.c_str());
    return;
  }

  RCLCPP_WARN(
    node->get_logger(),
    "SemanticZoneLayer: loaded %zu zones, bbox=[%.2f,%.2f]-[%.2f,%.2f], enabled=%d",
    zones_.size(), zone_min_x_, zone_min_y_, zone_max_x_, zone_max_y_, enabled_);
}

void SemanticZoneLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_ || !zones_loaded_) {
    return;
  }
  // Expand bounds to cover all zones
  *min_x = std::min(*min_x, zone_min_x_);
  *min_y = std::min(*min_y, zone_min_y_);
  *max_x = std::max(*max_x, zone_max_x_);
  *max_y = std::max(*max_y, zone_max_y_);
}

void SemanticZoneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !zones_loaded_) {
    return;
  }

  auto node = node_.lock();
  unsigned int cells_marked = 0;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      for (const auto & zone : zones_) {
        if (pointInPolygon(wx, wy, zone.vertices)) {
          unsigned char old_cost = master_grid.getCost(i, j);
          if (SPEED_BUMP_COST > old_cost) {
            master_grid.setCost(i, j, SPEED_BUMP_COST);
            ++cells_marked;
          }
        }
      }
    }
  }

  if (node && cells_marked > 0) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "SemanticZoneLayer::updateCosts: marked %u cells in [%d,%d]-[%d,%d]",
      cells_marked, min_i, min_j, max_i, max_j);
  }
}

// Ray-casting point-in-polygon test
bool SemanticZoneLayer::pointInPolygon(
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

}  // namespace pb_nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::SemanticZoneLayer, nav2_costmap_2d::Layer)
