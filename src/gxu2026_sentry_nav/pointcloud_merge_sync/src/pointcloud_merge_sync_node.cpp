// Time-synchronized PointCloud2 merger for dual LiDAR costmap input.
// Subscribes to two PointCloud2 streams, pairs them via ApproximateTime,
// concatenates, and publishes a single merged cloud.

#include <mutex>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace
{
constexpr double kMergeTimeToleranceSec = 0.1;
constexpr double kFallbackTimeoutSec = 0.5;
constexpr int kQueueSize = 10;
}  // namespace

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

class PointcloudMergeSync : public rclcpp::Node
{
public:
  PointcloudMergeSync()
  : Node("pointcloud_merge_sync")
  {
    const auto primary_topic =
        this->declare_parameter<std::string>("primary_topic", "registered_scan");
    const auto secondary_topic =
        this->declare_parameter<std::string>("secondary_topic", "mid360/registered_scan");
    const auto output_topic =
        this->declare_parameter<std::string>("output_topic", "merged_registered_scan");
    merge_time_tolerance_sec_ = this->declare_parameter<double>(
        "merge_time_tolerance_sec", kMergeTimeToleranceSec);
    const int queue_size =
        this->declare_parameter<int>("queue_size", kQueueSize);
    const double fallback_timeout_sec =
        this->declare_parameter<double>("fallback_timeout_sec", kFallbackTimeoutSec);

    const auto qos = rclcpp::SensorDataQoS();
    const auto rmw_qos = qos.get_rmw_qos_profile();

    output_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    primary_sub_.subscribe(this, primary_topic, rmw_qos);
    secondary_sub_.subscribe(this, secondary_topic, rmw_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), primary_sub_, secondary_sub_);
    sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(merge_time_tolerance_sec_));
    sync_->registerCallback(std::bind(
        &PointcloudMergeSync::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    primary_raw_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        primary_topic, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          last_primary_msg_ = std::move(msg);
        });

    fallback_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(fallback_timeout_sec), [this]() {
          std::lock_guard<std::mutex> lock(mutex_);
          if (last_primary_msg_ && !last_sync_success_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "No synchronized pair, fallback to primary only.");
            output_pub_->publish(*last_primary_msg_);
          }
          last_sync_success_ = false;
        });
  }

private:
  void syncCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & primary_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & secondary_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_sync_success_ = true;
    last_primary_msg_ = primary_msg;

    pcl::PointCloud<pcl::PointXYZI> primary_pcl, secondary_pcl;
    pcl::fromROSMsg(*primary_msg, primary_pcl);
    pcl::fromROSMsg(*secondary_msg, secondary_pcl);

    primary_pcl += secondary_pcl;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(primary_pcl, output);
    output.header.stamp = primary_msg->header.stamp;
    output.header.frame_id = primary_msg->header.frame_id;
    output_pub_->publish(output);
  }

  double merge_time_tolerance_sec_{kMergeTimeToleranceSec};
  bool last_sync_success_{false};

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> primary_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> secondary_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_raw_sub_;
  rclcpp::TimerBase::SharedPtr fallback_timer_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_primary_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudMergeSync>());
  rclcpp::shutdown();
  return 0;
}
