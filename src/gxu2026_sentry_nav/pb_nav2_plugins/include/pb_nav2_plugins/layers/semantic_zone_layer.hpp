// Copyright 2026 GXU RMUL Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef PB_NAV2_PLUGINS__LAYERS__SEMANTIC_ZONE_LAYER_HPP_
#define PB_NAV2_PLUGINS__LAYERS__SEMANTIC_ZONE_LAYER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb_nav2_costmap_2d
{

struct SemanticZone
{
  std::string name;
  std::string type;                            // speed_bump
  std::vector<std::pair<double, double>> vertices;  // polygon vertices in map frame
  unsigned char cost;                          // costmap cost value
};

class SemanticZoneLayer : public nav2_costmap_2d::Layer
{
public:
  SemanticZoneLayer() = default;
  ~SemanticZoneLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override {}
  bool isClearable() override { return false; }

private:
  void loadZonesFromYaml(const std::string & yaml_path);
  void publishZoneMarkers();
  bool pointInPolygon(
    double x, double y,
    const std::vector<std::pair<double, double>> & poly) const;

  std::string target_zone_name_{"speed_bump"};
  int slow_zone_cost_{180};

  bool publish_markers_{true};
  std::string marker_topic_{"semantic_zone_markers"};
  std::string marker_frame_id_{"map"};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::vector<SemanticZone> zones_;
  bool zones_loaded_{false};

  // Bounding box of all zones (world coords) for updateBounds
  double zone_min_x_{0.0}, zone_min_y_{0.0};
  double zone_max_x_{0.0}, zone_max_y_{0.0};
};

}  // namespace pb_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__SEMANTIC_ZONE_LAYER_HPP_
