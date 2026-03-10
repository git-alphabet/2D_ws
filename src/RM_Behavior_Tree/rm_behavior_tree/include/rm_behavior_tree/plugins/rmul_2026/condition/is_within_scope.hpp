#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_SCOPE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_SCOPE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

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
			BT::InputPort<double>("arrive_radius")
		};
	}

	BT::NodeStatus tick() override;

private:
	// 保留 params_ 与其它 ROS 节点构造签名一致
	BT::RosNodeParams params_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_WITHIN_SCOPE_HPP_

