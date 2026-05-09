// Time-synchronized PointCloud2 merger for dual LiDAR costmap input.
// Subscribes to two PointCloud2 streams, pairs them via ApproximateTime,
// concatenates, and publishes a single merged cloud.

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
    const double merge_time_tolerance_sec = this->declare_parameter<double>(
        "merge_time_tolerance_sec", kMergeTimeToleranceSec);
    const int queue_size =
        this->declare_parameter<int>("queue_size", kQueueSize);

    const auto qos = rclcpp::SensorDataQoS();
    const auto rmw_qos = qos.get_rmw_qos_profile();

    output_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    primary_sub_.subscribe(this, primary_topic, rmw_qos);
    secondary_sub_.subscribe(this, secondary_topic, rmw_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), primary_sub_, secondary_sub_);
    sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(merge_time_tolerance_sec));
    sync_->registerCallback(std::bind(
        &PointcloudMergeSync::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void syncCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & primary_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & secondary_msg)
  {
    if (primary_msg->data.empty()) {
      return;
    }

    if (secondary_msg->data.empty()) {
      output_pub_->publish(*primary_msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> primary_pcl, secondary_pcl;
    pcl::fromROSMsg(*primary_msg, primary_pcl);
    pcl::fromROSMsg(*secondary_msg, secondary_pcl);

    if (primary_pcl.empty() && secondary_pcl.empty()) {
      return;
    }

    primary_pcl += secondary_pcl;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(primary_pcl, output);
    output.header.stamp = primary_msg->header.stamp;
    output.header.frame_id = primary_msg->header.frame_id;
    output_pub_->publish(output);
  }

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> primary_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> secondary_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudMergeSync>());
  rclcpp::shutdown();
  return 0;
}
