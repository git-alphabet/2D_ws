#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_IN_SEMANTIC_ZONE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_IN_SEMANTIC_ZONE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

struct SemanticZoneDef
{
  std::string name;
  std::string type;
  std::vector<std::pair<double, double>> vertices;
};

/**
 * @brief 查询机器人是否在指定类型的语义区域内
 *
 * 从 semantic_zones.yaml 加载多边形区域，
 * 返回 SUCCESS 表示机器人在匹配 zone_type 的某个区域内。
 */
class IsInSemanticZoneCondition : public BT::ConditionNode
{
public:
  IsInSemanticZoneCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"),
      BT::InputPort<double>("pose_y"),
      BT::InputPort<std::string>("zone_type", "keepout",
        "Zone type to query: keepout / high_cost / spin_zone / slow_zone"),
      BT::InputPort<std::string>("zones_file", "",
        "Path to semantic_zones.yaml"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void loadZones(const std::string & yaml_path);
  bool pointInPolygon(
    double x, double y,
    const std::vector<std::pair<double, double>> & poly) const;

  std::vector<SemanticZoneDef> zones_;
  bool loaded_{false};
  std::string loaded_file_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_IN_SEMANTIC_ZONE_HPP_
