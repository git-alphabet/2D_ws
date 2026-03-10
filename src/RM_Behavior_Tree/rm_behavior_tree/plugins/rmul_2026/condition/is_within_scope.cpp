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
: BT::ConditionNode(name, conf), params_(params)
{
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
	// TODO: 将下面的占位常量替换为正确的补给区坐标或通过外部配置注入
	constexpr double kDefaultGoalX = 0.0;  // TODO: 补给区 X 坐标占位
	constexpr double kDefaultGoalY = 0.0;  // TODO: 补给区 Y 坐标占位
	constexpr double kDefaultArriveRadius = 0.5;  // 默认有效半径 (m)

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

	const double dx = pose_x - goal_x;
	const double dy = pose_y - goal_y;
	const double dist = std::hypot(dx, dy);

	return (dist <= arrive_radius) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

// 导出为插件，名称与 XML 中使用的节点名一致
CreateRosNodePlugin(rm_behavior_tree::IsWithinScopeCondition, "IsWithinScope");
