#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__LOAD_CALIBRATION_CSV_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__LOAD_CALIBRATION_CSV_HPP_

#include <string>
#include <filesystem>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

/// 从 CSV 文件加载标定坐标，覆盖 InitSentryConfig 设置的 YAML 默认值
/// CSV 格式: 点位名称,x,y (# 开头为注释)
/// 巡逻航点: patrol_1,x,y  patrol_2,x,y  ...
/// 若文件不存在或点位缺失，保持原有 YAML 默认值
class LoadCalibrationCSVAction : public BT::SyncActionNode
{
public:
  LoadCalibrationCSVAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("csv_path", "", "标定 CSV 文件路径 (空则跳过)"),
      // 坐标输出端口 (与 InitSentryConfig 对齐)
      BT::OutputPort<double>("home_x"), BT::OutputPort<double>("home_y"),
      BT::OutputPort<double>("supply_zone_x"), BT::OutputPort<double>("supply_zone_y"),
      BT::OutputPort<double>("base_buff_x"), BT::OutputPort<double>("base_buff_y"),
      BT::OutputPort<double>("outpost_buff_x"), BT::OutputPort<double>("outpost_buff_y"),
      BT::OutputPort<double>("fortress_ally_x"), BT::OutputPort<double>("fortress_ally_y"),
      BT::OutputPort<double>("fortress_enemy_x"), BT::OutputPort<double>("fortress_enemy_y"),
      BT::OutputPort<double>("central_highland_x"), BT::OutputPort<double>("central_highland_y"),
      BT::OutputPort<double>("ladder_highland_x"), BT::OutputPort<double>("ladder_highland_y"),
      BT::OutputPort<double>("defend_anchor_x"), BT::OutputPort<double>("defend_anchor_y"),
      BT::OutputPort<double>("central_highland_left_x"), BT::OutputPort<double>("central_highland_left_y"),
      BT::OutputPort<double>("ramp_jump_x"), BT::OutputPort<double>("ramp_jump_y"),
      // 巡逻航点 (格式: "x1,y1;x2,y2;...")
      BT::OutputPort<std::string>("patrol_waypoints"),
    };
  }

  BT::NodeStatus tick() override;

private:
  bool logged_empty_path_{false};
  bool logged_open_fail_{false};
  bool loaded_once_{false};
  std::filesystem::file_time_type last_mtime_{};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__LOAD_CALIBRATION_CSV_HPP_
