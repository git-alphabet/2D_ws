#include "rm_behavior_tree/plugins/rmul_2026/condition/is_within_scope.hpp"

#include <cmath>
#include <optional>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

IsWithinScopeCondition::IsWithinScopeCondition(
	const std::string & name,
	const BT::NodeConfig & conf,
	const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), node_(params.nh)
{
}

void IsWithinScopeCondition::planCallback(
	const nav_msgs::msg::Path::SharedPtr msg)
{
	if (!msg || msg->poses.empty()) {
		return;
	}

	// 计算路径总长度
	double length = 0.0;
	for (size_t i = 1; i < msg->poses.size(); ++i) {
		const double dx =
			msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x;
		const double dy =
			msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y;
		length += std::hypot(dx, dy);
	}

	const auto & last = msg->poses.back().pose.position;

	std::lock_guard<std::mutex> lock(plan_mutex_);
	cached_path_length_ = length;
	cached_plan_goal_x_ = last.x;
	cached_plan_goal_y_ = last.y;
}

BT::NodeStatus IsWithinScopeCondition::tick()
{
	// 尝试从黑板读取机器人位姿
	auto res_x = getInput<double>("pose_x");
	auto res_y = getInput<double>("pose_y");

	if (!res_x || !res_y) {
		// 若没有位置信息，保守认为尚未到达有效半径
		return BT::NodeStatus::FAILURE;
	}

	double pose_x = res_x.value();
	double pose_y = res_y.value();

	// 尝试读取目标点和半径；如果没有提供，则使用占位值
	constexpr double kDefaultGoalX = 0.0;
	constexpr double kDefaultGoalY = 0.0;
	constexpr double kDefaultArriveRadius = 0.5;

	double goal_x = kDefaultGoalX;
	double goal_y = kDefaultGoalY;
	double arrive_radius = kDefaultArriveRadius;

	if (auto rgx = getInput<double>("goal_x"); rgx) {
		goal_x = rgx.value();
	}
	if (auto rgy = getInput<double>("goal_y"); rgy) {
		goal_y = rgy.value();
	}
	if (auto rrad = getInput<double>("arrive_radius"); rrad) {
		arrive_radius = rrad.value();
	}

	// ---- path distance 模式 ----
	bool use_path_distance = false;
	getInput("use_path_distance", use_path_distance);

	if (use_path_distance && node_) {
		// 惰性订阅 plan topic
		std::string plan_topic = "plan";
		getInput("plan_topic", plan_topic);

		if (!plan_sub_ || subscribed_plan_topic_ != plan_topic) {
			plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
				plan_topic, rclcpp::QoS(1),
				[this](const nav_msgs::msg::Path::SharedPtr msg) {
					planCallback(msg);
				});
			subscribed_plan_topic_ = plan_topic;
		}

		rclcpp::spin_some(node_);

		double goal_match_tol = 1.0;
		getInput("goal_match_tolerance", goal_match_tol);

		{
			std::lock_guard<std::mutex> lock(plan_mutex_);
			if (cached_path_length_ >= 0.0) {
				const double gdist = std::hypot(
					cached_plan_goal_x_ - goal_x,
					cached_plan_goal_y_ - goal_y);
				if (gdist <= goal_match_tol) {
					// plan 终点匹配我们的 goal，使用路径距离
					return (cached_path_length_ <= arrive_radius)
						? BT::NodeStatus::SUCCESS
						: BT::NodeStatus::FAILURE;
				}
			}
		}
		// plan 终点不匹配或无缓存 → 回退到欧氏距离
	}

	// ---- 欧氏距离模式（默认） ----
	const double dx = pose_x - goal_x;
	const double dy = pose_y - goal_y;
	const double dist = std::hypot(dx, dy);

	return (dist <= arrive_radius) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

// 导出为插件，名称与 XML 中使用的节点名一致
CreateRosNodePlugin(rm_behavior_tree::IsWithinScopeCondition, "IsWithinScope");
