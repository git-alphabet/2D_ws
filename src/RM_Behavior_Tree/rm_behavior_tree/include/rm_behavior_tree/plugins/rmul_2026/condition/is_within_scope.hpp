#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_SCOPE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_SCOPE_HPP_

#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class IsWithinScopeCondition : public BT::ConditionNode
{
public:
	IsWithinScopeCondition(
		const std::string & name,
		const BT::NodeConfig & conf,
		const BT::RosNodeParams & params);

	// 声明端口：机器人位姿、目标点和有效半径
	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<double>("pose_x"),
			BT::InputPort<double>("pose_y"),
			// 可由外部配置传入；如果外部没有提供，节点会使用内部占位值（在实现中注明）
			BT::InputPort<double>("goal_x"),
			BT::InputPort<double>("goal_y"),
			BT::InputPort<double>("arrive_radius"),
			BT::InputPort<bool>("use_path_distance", false,
				"true=use global plan path length instead of Euclidean distance"),
			BT::InputPort<std::string>("plan_topic", "plan",
				"topic name for nav_msgs/Path (global plan)"),
			BT::InputPort<double>("goal_match_tolerance", 1.0,
				"max distance between plan endpoint and goal to consider them matching"),
		};
	}

	BT::NodeStatus tick() override;

private:
	rclcpp::Node::SharedPtr node_;

	// ---- path distance 相关 ----
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
	std::string subscribed_plan_topic_;
	mutable std::mutex plan_mutex_;
	double cached_path_length_ = -1.0;   // <0 表示尚无有效缓存
	double cached_plan_goal_x_ = 0.0;
	double cached_plan_goal_y_ = 0.0;

	void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_SCOPE_HPP_

