#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  const bool use_inf,
  const bool in_primary_fov)
{
  const bool primary_valid = isRangeValid(primary, range_min, range_max);
  const bool secondary_valid = isRangeValid(secondary, range_min, range_max);

  if (in_primary_fov) {
    if (primary_valid) {
      return primary;
    }
  } else {
    if (primary_valid && secondary_valid) {
      return std::min(primary, secondary);
    }
  }

  if (primary_valid) {
    return primary;
  }
  if (secondary_valid) {
    return secondary;
  }

  return use_inf ? std::numeric_limits<float>::infinity() : range_max;
}

constexpr double kMergeTimeToleranceSec = 0.05;

}  // namespace

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

class ScanAdditiveAdapter : public rclcpp::Node
{
public:
  ScanAdditiveAdapter()
  : Node("scan_additive_adapter")
  {
    const auto primary_topic = this->declare_parameter<std::string>(
      "primary_scan_topic", "scan_odin1");
    const auto secondary_topic = this->declare_parameter<std::string>(
      "secondary_scan_topic", "scan_mid360");
    const auto output_topic = this->declare_parameter<std::string>(
      "output_scan_topic", "obstacle_scan");

    primary_fov_min_ = this->declare_parameter<double>(
      "primary_fov_angle_min", -5.0 * M_PI / 6.0);
    primary_fov_max_ = this->declare_parameter<double>(
      "primary_fov_angle_max", -M_PI / 6.0);

    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");

    merge_time_tolerance_sec_ = this->declare_parameter<double>(
      "merge_time_tolerance_sec", kMergeTimeToleranceSec);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    tf_buffer_->setUsingDedicatedThread(true);

    const auto qos = rclcpp::SensorDataQoS();
    const auto rmw_qos = qos.get_rmw_qos_profile();

    output_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos);
    diag_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/correction_diag", qos);

    // ApproximateTime synchronizer: only merge when timestamps are close.
    primary_sub_.subscribe(this, primary_topic, rmw_qos);
    secondary_sub_.subscribe(this, secondary_topic, rmw_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), primary_sub_, secondary_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(merge_time_tolerance_sec_));
    sync_->registerCallback(
      std::bind(&ScanAdditiveAdapter::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Standalone primary subscription for fallback and diagnostic.
    primary_raw_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      primary_topic, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_primary_msg_ = std::move(msg);
        last_primary_mono_ = std::chrono::steady_clock::now();
      });

    // Fallback timer: if no sync pair for 0.5s, publish primary-only.
    fallback_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(0.5),
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_primary_msg_ && !last_sync_success_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "No synchronized pair, fallback to primary only.");
          output_pub_->publish(*last_primary_msg_);
          publishDiag(0, 0, 0, false, 0);
        }
      });
  }

private:
  void syncCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & primary_msg,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & secondary_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_sync_success_ = true;

    const auto primary_stamp = rclcpp::Time(primary_msg->header.stamp);
    const auto secondary_stamp = rclcpp::Time(secondary_msg->header.stamp);
    const double dt = (primary_stamp - secondary_stamp).seconds();

    // TF-correct secondary to primary's timestamp.
    sensor_msgs::msg::LaserScan secondary_corrected =
      transformScanToTime(*secondary_msg, primary_stamp);

    // Merge: primary as base, secondary for complement.
    sensor_msgs::msg::LaserScan merged = *primary_msg;
    const bool use_inf = merged.range_max > merged.range_min;
    const float angle_min = merged.angle_min;
    const float angle_inc = merged.angle_increment;

    for (size_t i = 0; i < merged.ranges.size(); ++i) {
      const float angle = angle_min + static_cast<float>(i) * angle_inc;
      const bool in_primary_fov = (angle >= primary_fov_min_ && angle <= primary_fov_max_);

      merged.ranges[i] = mergeRange(
        primary_msg->ranges[i],
        secondary_corrected.ranges[i],
        merged.range_min,
        merged.range_max,
        use_inf,
        in_primary_fov);
    }

    output_pub_->publish(merged);
    publishDiag(last_dx_, last_dy_, last_dyaw_, last_tf_ok_, dt);
  }

  bool computeRelativeTransform(
    const rclcpp::Time & target_time,
    const rclcpp::Time & scan_time,
    double & dx, double & dy, double & dyaw)
  {
    if (target_time == scan_time) {
      dx = 0.0; dy = 0.0; dyaw = 0.0;
      return true;
    }

    geometry_msgs::msg::TransformStamped tf_target, tf_scan;
    try {
      if (!tf_buffer_->canTransform(
            odom_frame_, base_frame_, target_time, tf2::durationFromSec(0.0)) ||
          !tf_buffer_->canTransform(
            odom_frame_, base_frame_, scan_time, tf2::durationFromSec(0.0)))
      {
        return false;
      }
      tf_target = tf_buffer_->lookupTransform(odom_frame_, base_frame_, target_time);
      tf_scan = tf_buffer_->lookupTransform(odom_frame_, base_frame_, scan_time);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed (%s->%s): %s",
        odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    const auto yawFromQuat = [](const geometry_msgs::msg::Quaternion & q) -> double {
      return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    };

    const double yaw_t = yawFromQuat(tf_target.transform.rotation);
    const double yaw_s = yawFromQuat(tf_scan.transform.rotation);
    const double tx_t = tf_target.transform.translation.x;
    const double ty_t = tf_target.transform.translation.y;
    const double tx_s = tf_scan.transform.translation.x;
    const double ty_s = tf_scan.transform.translation.y;

    // T_rel = T_scan^{-1} * T_target
    dyaw = yaw_t - yaw_s;
    const double cos_ys = std::cos(yaw_s);
    const double sin_ys = std::sin(yaw_s);
    const double dtx = tx_t - tx_s;
    const double dty = ty_t - ty_s;
    dx = cos_ys * dtx + sin_ys * dty;
    dy = -sin_ys * dtx + cos_ys * dty;

    return true;
  }

  sensor_msgs::msg::LaserScan transformScanToTime(
    const sensor_msgs::msg::LaserScan & scan,
    const rclcpp::Time & target_time)
  {
    sensor_msgs::msg::LaserScan result = scan;

    double dx = 0.0, dy = 0.0, dyaw = 0.0;
    if (!computeRelativeTransform(target_time, rclcpp::Time(scan.header.stamp), dx, dy, dyaw))
    {
      last_tf_ok_ = false;
      return result;
    }

    last_dx_ = dx;
    last_dy_ = dy;
    last_dyaw_ = dyaw;
    last_tf_ok_ = true;

    if (std::abs(dx) < 1e-4 && std::abs(dy) < 1e-4 && std::abs(dyaw) < 1e-4) {
      result.header.stamp = target_time;
      return result;
    }

    const float r_min = scan.range_min;
    const float r_max = scan.range_max;
    const size_t n = scan.ranges.size();
    const float a_inc = scan.angle_increment;
    const float a_min = scan.angle_min;
    const float a_max = scan.angle_max;
    const double cos_d = std::cos(dyaw);
    const double sin_d = std::sin(dyaw);

    for (size_t i = 0; i < n; ++i) {
      if (!isRangeValid(scan.ranges[i], r_min, r_max)) {
        continue;
      }
      const float angle = a_min + static_cast<float>(i) * a_inc;
      const double x = scan.ranges[i] * std::cos(angle);
      const double y = scan.ranges[i] * std::sin(angle);

      const double x_new = x * cos_d - y * sin_d + dx;
      const double y_new = x * sin_d + y * cos_d + dy;

      const float r_new = static_cast<float>(std::hypot(x_new, y_new));
      if (r_new < r_min || r_new > r_max) {
        continue;
      }

      const float a_new = static_cast<float>(std::atan2(y_new, x_new));
      if (a_new < a_min || a_new > a_max) {
        continue;
      }
      const int j = static_cast<int>(std::round((a_new - a_min) / a_inc));
      if (j < 0 || j >= static_cast<int>(n)) {
        continue;
      }

      if (!std::isfinite(result.ranges[j]) || r_new < result.ranges[j]) {
        result.ranges[j] = r_new;
      }
    }

    result.header.stamp = target_time;
    return result;
  }

  void publishDiag(double dx, double dy, double dyaw, bool tf_ok, double dt)
  {
    std_msgs::msg::Float64MultiArray diag;
    diag.data = {dx, dy, dyaw, tf_ok ? 1.0 : 0.0, dt};
    diag_pub_->publish(diag);
  }

  double merge_time_tolerance_sec_{kMergeTimeToleranceSec};
  double primary_fov_min_{-5.0 * M_PI / 6.0};
  double primary_fov_max_{-M_PI / 6.0};

  std::string odom_frame_;
  std::string base_frame_;

  double last_dx_{0.0};
  double last_dy_{0.0};
  double last_dyaw_{0.0};
  bool last_tf_ok_{false};
  bool last_sync_success_{false};

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Synchronized subscription
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> primary_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> secondary_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Standalone primary for fallback
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr primary_raw_sub_;
  rclcpp::TimerBase::SharedPtr fallback_timer_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_primary_msg_;
  std::chrono::steady_clock::time_point last_primary_mono_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanAdditiveAdapter>());
  rclcpp::shutdown();
  return 0;
}
