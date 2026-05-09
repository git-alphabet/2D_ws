// Time-synchronized PointCloud2 merger for dual LiDAR costmap input.
// Subscribes to two PointCloud2 streams, pairs them via ApproximateTime,
// concatenates, and publishes a single merged cloud.
// Always publishes primary as baseline; sync overrides when both available.
// Normalizes all output to XYZI format.

#include <chrono>
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
constexpr double kMergeTimeToleranceSec = 0.08;
constexpr int kQueueSize = 20;
constexpr double kSyncSuppressionSec = 0.3;
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
    const double merge_time_tolerance_sec = this->declare_parameter<double>(
        "merge_time_tolerance_sec", kMergeTimeToleranceSec);
    const int queue_size =
        this->declare_parameter<int>("queue_size", kQueueSize);

    const auto qos = rclcpp::SensorDataQoS();
    const auto rmw_qos = qos.get_rmw_qos_profile();

    output_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    // Dual-source sync path
    primary_sub_.subscribe(this, primary_topic, rmw_qos);
    secondary_sub_.subscribe(this, secondary_topic, rmw_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), primary_sub_, secondary_sub_);
    sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(merge_time_tolerance_sec));
    sync_->registerCallback(std::bind(
        &PointcloudMergeSync::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Single-source fallback: always active, suppressed briefly when sync fires
    primary_direct_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        primary_topic, qos,
        std::bind(&PointcloudMergeSync::primaryDirectCallback, this, std::placeholders::_1));

    // Initialize to well in the past so fallback is immediately active
    last_sync_stamp_ = this->now() - rclcpp::Duration::from_seconds(10.0);

    RCLCPP_INFO(
        this->get_logger(),
        "primary=%s secondary=%s output=%s tolerance=%.3fs queue=%d",
        primary_topic.c_str(), secondary_topic.c_str(),
        output_topic.c_str(), merge_time_tolerance_sec, queue_size);
  }

private:
  static pcl::PointCloud<pcl::PointXYZI> toXYZI(
      const sensor_msgs::msg::PointCloud2 & msg)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;

    bool has_intensity = false;
    for (const auto & field : msg.fields) {
      if (field.name == "intensity") {
        has_intensity = true;
        break;
      }
    }

    if (has_intensity) {
      pcl::fromROSMsg(msg, cloud);
    } else {
      pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
      pcl::fromROSMsg(msg, xyz_cloud);
      cloud.resize(xyz_cloud.size());
      for (size_t i = 0; i < xyz_cloud.size(); ++i) {
        cloud[i].x = xyz_cloud[i].x;
        cloud[i].y = xyz_cloud[i].y;
        cloud[i].z = xyz_cloud[i].z;
        cloud[i].intensity = 0.0f;
      }
    }
    return cloud;
  }

  static sensor_msgs::msg::PointCloud2 toXYZIMsg(
      const pcl::PointCloud<pcl::PointXYZI> & cloud,
      const std_msgs::msg::Header & header)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    return msg;
  }

  // Sync path: both sources matched — publish merged cloud
  void syncCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & primary_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & secondary_msg)
  {
    last_sync_stamp_ = this->now();

    if (primary_msg->data.empty()) {
      return;
    }

    if (secondary_msg->data.empty()) {
      auto cloud = toXYZI(*primary_msg);
      output_pub_->publish(toXYZIMsg(cloud, primary_msg->header));
      return;
    }

    auto primary_pcl = toXYZI(*primary_msg);
    auto secondary_pcl = toXYZI(*secondary_msg);

    if (primary_pcl.empty() && secondary_pcl.empty()) {
      return;
    }

    primary_pcl += secondary_pcl;

    std_msgs::msg::Header header;
    header.stamp = primary_msg->header.stamp;
    header.frame_id = primary_msg->header.frame_id;
    output_pub_->publish(toXYZIMsg(primary_pcl, header));
  }

  // Fallback path: publish primary-only when sync is not producing output
  void primaryDirectCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    // Suppress fallback briefly when sync is active to avoid duplicates
    const double since_sync = (this->now() - last_sync_stamp_).seconds();
    if (since_sync < kSyncSuppressionSec) {
      return;
    }
    if (msg->data.empty()) {
      return;
    }
    auto cloud = toXYZI(*msg);
    output_pub_->publish(toXYZIMsg(cloud, msg->header));
  }

  // Dual-source sync
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> primary_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> secondary_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Single-source fallback
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_direct_sub_;

  // Timestamp of last successful sync
  rclcpp::Time last_sync_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudMergeSync>());
  rclcpp::shutdown();
  return 0;
}
