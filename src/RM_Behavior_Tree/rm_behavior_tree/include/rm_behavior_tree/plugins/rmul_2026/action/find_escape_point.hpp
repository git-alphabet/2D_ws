#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_ESCAPE_POINT_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_ESCAPE_POINT_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * 卡住脱困专用插件：从机器人当前位置搜索安全逃脱点。
 *
 * 三阶段搜索策略：
 *   ① cost < 50,  半径 0.5~2.0m, 优先后退方向
 *   ② cost < 150, 半径 0.5~3.0m, 优先后退方向
 *   ③ cost < 235, 半径 0.5~3.0m, 任意方向
 *
 * "后退方向" = normalize(robot - goal)，即远离目标的方向。
 *
 * 状态保持：一旦选定逃脱点，后续 tick 返回同一个点，直到：
 *   - 该点代价值升高变为障碍（cost >= 235）
 *   - 机器人距该点 > max_commit_distance（5.0m，说明已不在脱困流程中）
 *
 * 输入：robot_x, robot_y (机器人位置), goal_x, goal_y (原始目标)
 * 输出：escape_x, escape_y, escape_pose (逃脱点)
 * 返回：SUCCESS=找到/已有逃脱点  FAILURE=三阶段均无可用点
 */
class FindEscapePointAction : public BT::SyncActionNode
{
public:
  FindEscapePointAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("costmap_topic", "global_costmap/costmap_raw",
                                 "costmap topic to subscribe"),
      BT::InputPort<double>("robot_x", "robot current x"),
      BT::InputPort<double>("robot_y", "robot current y"),
      BT::InputPort<double>("goal_x", "original goal x"),
      BT::InputPort<double>("goal_y", "original goal y"),
      BT::InputPort<int>("escape_timeout_ms", 0,
                          "max ms to stay at escape point before returning FAILURE (0=no timeout)"),
      BT::InputPort<double>("arrive_radius", 0.3,
                             "distance to escape point considered arrived"),
      BT::OutputPort<double>("escape_x", "escape point x"),
      BT::OutputPort<double>("escape_y", "escape point y"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("escape_pose", "escape point as PoseStamped"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  std::string subscribed_topic_;

  mutable std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

  /// 查询网格代价值；越界返回 255
  uint8_t getCost(double world_x, double world_y) const;

  /// 计算某点周围1格邻域的平均代价（clearance bonus 用）
  double getNeighborClearance(double wx, double wy, double step) const;

  struct SearchParams {
    double min_radius;
    double max_radius;
    double step;
    uint8_t cost_threshold;
    bool prefer_retreat;  // true=优先后退方向
  };

  struct CandidatePoint {
    double x;
    double y;
    double dist_to_goal;
    double retreat_dot;    // >0 表示在后退方向
    double clearance;      // 周围平均空闲度 (253 - avg_cost)
  };

  /// 在给定参数下搜索候选点，返回最佳逃脱点
  bool searchPhase(
    double robot_x, double robot_y,
    double goal_x, double goal_y,
    double retreat_dx, double retreat_dy,
    const SearchParams & params,
    double & out_x, double & out_y) const;

  /// 输出逃脱点到端口
  void outputResult(double x, double y);

  /// 提交并输出逃脱点（记录状态 + 日志）
  void commitAndOutput(double x, double y, const char * phase, double robot_x, double robot_y);

  // ---- 状态保持 ----
  bool has_committed_ = false;
  double committed_x_ = 0.0;
  double committed_y_ = 0.0;
  static constexpr double MAX_COMMIT_DISTANCE = 5.0;  // 超过此距离认为已离开脱困流程

  // ---- 到达脱困点计时（超时后返回FAILURE，用于补给区紧急场景） ----
  bool at_escape_ = false;
  bool timed_out_ = false;  // 超时后持续返回FAILURE，冷却2秒后自动重置
  std::chrono::steady_clock::time_point escape_arrival_time_;
  std::chrono::steady_clock::time_point timeout_set_time_;  // timed_out_置位的时刻

  // ---- 心跳检测（替代 status()==IDLE，因为 SyncActionNode::halt() 是 final） ----
  bool last_tick_time_set_ = false;
  std::chrono::steady_clock::time_point last_tick_time_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_ESCAPE_POINT_HPP_
