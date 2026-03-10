#include "rm_behavior_tree/plugins/rmul_2026/action/cancel_nav_goal.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include <type_traits>

namespace rm_behavior_tree
{

static inline std::string trim_copy(std::string s)
{
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

CancelNavGoalAction::CancelNavGoalAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  // Some codebases provide params.nh as shared_ptr, some as weak_ptr.
  // Use a small, compile-time-safe helper to obtain rclcpp::Node::SharedPtr.
  auto get_node = [](const auto & nh) -> rclcpp::Node::SharedPtr {
    using NH = std::decay_t<decltype(nh)>;
    if constexpr (std::is_convertible_v<NH, rclcpp::Node::SharedPtr>) {
      return nh;
    } else {
      return nh.lock();
    }
  };

  node_ = get_node(params.nh);
  if (!node_) {
    throw BT::RuntimeError("CancelNavGoalAction: failed to obtain ROS node from params.nh");
  }
}

std::string CancelNavGoalAction::makeCancelServiceName_(std::string action_name)
{
  action_name = trim_copy(action_name);
  if (action_name.empty()) {
    action_name = "navigate_to_pose";
  }

  // 如果用户已经填了完整 cancel service 名字，就原样用（容错）
  const std::string suffix = "/_action/cancel_goal";
  if (action_name.size() >= suffix.size() &&
      action_name.compare(action_name.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    if (!action_name.empty() && action_name.front() != '/') {
      action_name = "/" + action_name;
    }
    return action_name;
  }

  // 去掉末尾 '/'
  while (!action_name.empty() && action_name.back() == '/') {
    action_name.pop_back();
  }
  // 确保以 '/' 开头
  if (!action_name.empty() && action_name.front() != '/') {
    action_name = "/" + action_name;
  }

  return action_name + suffix;
}

BT::NodeStatus CancelNavGoalAction::tick()
{
  // 1) 读取 ports
  std::string action_name = "navigate_to_pose";
  if (auto r = getInput<std::string>("action_name")) {
    action_name = r.value();
  }

  int min_interval_ms = 0;
  if (auto r = getInput<int>("min_interval_ms")) {
    min_interval_ms = std::max(0, r.value());
  }

  int wait_for_service_ms = 0;
  if (auto r = getInput<int>("wait_for_service_ms")) {
    wait_for_service_ms = std::max(0, r.value());
  }

  bool fail_on_unavailable = false;
  if (auto r = getInput<bool>("fail_on_unavailable")) {
    fail_on_unavailable = r.value();
  }

  int wait_response_ms = 0;
  if (auto r = getInput<int>("wait_response_ms")) {
    wait_response_ms = std::max(0, r.value());
  }

  // 2) 节流：避免频繁 cancel
  auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  if (has_last_call_ && min_interval_ms > 0) {
    const int64_t dt_ns = (now - last_call_time_).nanoseconds();
    const int64_t min_dt_ns = static_cast<int64_t>(min_interval_ms) * 1000000LL;
    if (dt_ns >= 0 && dt_ns < min_dt_ns) {
      // 节流期间：直接认为已完成（不再发 cancel）
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 3) 计算 cancel service name，并（必要时）重建 client
  const std::string new_service = makeCancelServiceName_(action_name);
  if (!client_ || new_service != service_name_) {
    service_name_ = new_service;
    client_ = node_->create_client<action_msgs::srv::CancelGoal>(service_name_);
  }

  // 4) 检查 service 是否可用
  bool service_ready = false;
  if (wait_for_service_ms > 0) {
    service_ready = client_->wait_for_service(std::chrono::milliseconds(wait_for_service_ms));
  } else {
    service_ready = client_->wait_for_service(std::chrono::milliseconds(0));
  }

  if (!service_ready) {
    RCLCPP_WARN(
      node_->get_logger(),
      "CancelNavGoal: service not available: %s",
      service_name_.c_str());

    if (fail_on_unavailable) {
      return BT::NodeStatus::FAILURE;
    }
    // 没有可取消的 action server 时，不阻塞树
    return BT::NodeStatus::SUCCESS;
  }

  // 5) 构造 CancelGoal 请求：
  //    goal_id 全 0 + stamp=0 => cancel all goals :contentReference[oaicite:1]{index=1}
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  std::fill(request->goal_info.goal_id.uuid.begin(),
            request->goal_info.goal_id.uuid.end(), 0);
  request->goal_info.stamp.sec = 0;
  request->goal_info.stamp.nanosec = 0;

  // debug 输出（可按需要删）
  std::cout << "[CancelNavGoal] call " << service_name_
            << " (cancel_all) @ " << now.seconds() << "s\n";

  // 6) 发送请求
  auto future = client_->async_send_request(request);

  // 记录节流时间
  last_call_time_ = now;
  has_last_call_ = true;

  // 7) 可选：等待响应（用于拿 return_code）
  if (wait_response_ms > 0) {
    auto status = future.wait_for(std::chrono::milliseconds(wait_response_ms));
    if (status == std::future_status::ready) {
      auto response = future.get();
      // 输出 return_code（0=ERROR_NONE 等，详见 action_msgs/srv/CancelGoal）:contentReference[oaicite:2]{index=2}
      setOutput<int>("return_code", static_cast<int>(response->return_code));

      // 一般来说：只要发出 cancel 就算成功；如需严格，可在这里判断 return_code
      return BT::NodeStatus::SUCCESS;
    } else {
      // 超时没等到响应，不影响主逻辑
      setOutput<int>("return_code", -1);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 不等待响应：fire-and-forget
  setOutput<int>("return_code", -1);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::CancelNavGoalAction, "CancelNavGoal");
