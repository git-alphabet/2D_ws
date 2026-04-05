#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

bool isRangeValid(const float value, const float range_min, const float range_max)
{
  return std::isfinite(value) && value >= range_min && value <= range_max;
}

float mergeRange(
  const float primary,
  const float secondary,
  const float range_min,
  const float range_max,
  const bool use_inf)
{
  const bool primary_valid = isRangeValid(primary, range_min, range_max);
  const bool secondary_valid = isRangeValid(secondary, range_min, range_max);

  if (primary_valid && secondary_valid) {
    return std::min(primary, secondary);
  }
  if (primary_valid) {
    return primary;
  }
  if (secondary_valid) {
    return secondary;
  }

  return use_inf ? std::numeric_limits<float>::infinity() : range_max;
}

}  // namespace

class ScanAdditiveAdapter : public rclcpp::Node
{
public:
  ScanAdditiveAdapter()
  : Node("scan_additive_adapter")
  {
    const auto primary_topic = this->declare_parameter<std::string>("primary_scan_topic", "scan_odin1");
    const auto secondary_topic = this->declare_parameter<std::string>("secondary_scan_topic", "scan_mid360");
    const auto output_topic = this->declare_parameter<std::string>("output_scan_topic", "obstacle_scan");

    secondary_timeout_sec_ = this->declare_parameter<double>("secondary_timeout_sec", 0.3);
    publish_secondary_when_primary_missing_ =
      this->declare_parameter<bool>("publish_secondary_when_primary_missing", true);

    const auto qos = rclcpp::SensorDataQoS();

    output_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos);

    primary_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      primary_topic,
      qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_primary_ = std::move(msg);
        publishMergedLocked();
      });

    secondary_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      secondary_topic,
      qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_secondary_ = std::move(msg);
        publishMergedLocked();
      });
  }

private:
  bool areScansCompatible(
    const sensor_msgs::msg::LaserScan & a,
    const sensor_msgs::msg::LaserScan & b) const
  {
    constexpr float kTol = 1e-5f;

    if (a.ranges.size() != b.ranges.size()) {
      return false;
    }

    if (std::fabs(a.angle_min - b.angle_min) > kTol) {
      return false;
    }
    if (std::fabs(a.angle_max - b.angle_max) > kTol) {
      return false;
    }
    if (std::fabs(a.angle_increment - b.angle_increment) > kTol) {
      return false;
    }

    return true;
  }

  bool isSecondaryFresh(const rclcpp::Time & now) const
  {
    if (!latest_secondary_) {
      return false;
    }

    const auto stamp = rclcpp::Time(latest_secondary_->header.stamp);
    const double age = (now - stamp).seconds();
    return age <= secondary_timeout_sec_;
  }

  void publishMergedLocked()
  {
    if (!latest_primary_ && !latest_secondary_) {
      return;
    }

    const auto now = this->get_clock()->now();

    if (!latest_primary_) {
      if (publish_secondary_when_primary_missing_ && latest_secondary_) {
        output_pub_->publish(*latest_secondary_);
      }
      return;
    }

    sensor_msgs::msg::LaserScan merged = *latest_primary_;

    if (!latest_secondary_ || !isSecondaryFresh(now)) {
      output_pub_->publish(merged);
      return;
    }

    if (!areScansCompatible(*latest_primary_, *latest_secondary_)) {
      output_pub_->publish(merged);
      return;
    }

    const bool use_inf = latest_primary_->range_max > latest_primary_->range_min;

    for (size_t i = 0; i < merged.ranges.size(); ++i) {
      merged.ranges[i] = mergeRange(
        latest_primary_->ranges[i],
        latest_secondary_->ranges[i],
        merged.range_min,
        merged.range_max,
        use_inf);
    }

    const auto primary_stamp = rclcpp::Time(latest_primary_->header.stamp);
    const auto secondary_stamp = rclcpp::Time(latest_secondary_->header.stamp);
    if (secondary_stamp > primary_stamp) {
      merged.header.stamp = latest_secondary_->header.stamp;
    }

    output_pub_->publish(merged);
  }

  std::mutex mutex_;

  double secondary_timeout_sec_{0.3};
  bool publish_secondary_when_primary_missing_{true};

  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_primary_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_secondary_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr primary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr secondary_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanAdditiveAdapter>());
  rclcpp::shutdown();
  return 0;
}
