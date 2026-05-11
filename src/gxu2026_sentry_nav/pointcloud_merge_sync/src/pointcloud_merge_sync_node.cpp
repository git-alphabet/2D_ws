// Time-synchronized PointCloud2 merger for dual LiDAR costmap input.
// Subscribes to two PointCloud2 streams, pairs them via ApproximateTime,
// concatenates, and publishes a single merged cloud.
// Always publishes primary as baseline; sync overrides when both available.
// Normalizes all output to XYZI format.

#include <chrono>
#include <string>

#include <Eigen/Dense>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kMergeTimeToleranceSec = 0.08;
constexpr int kQueueSize = 20;
constexpr double kSyncSuppressionSec = 0.3;
}  // namespace

using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
using ExactSyncPolicy = message_filters::sync_policies::ExactTime<
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
    const auto sync_policy =
        this->declare_parameter<std::string>("sync_policy", "approximate");
    const bool enable_fallback =
        this->declare_parameter<bool>("enable_fallback", true);
    enable_time_compensation_ =
      this->declare_parameter<bool>("enable_time_compensation", false);
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    max_time_compensation_sec_ =
      this->declare_parameter<double>("max_time_compensation_sec", 0.2);

    const auto qos = rclcpp::SensorDataQoS();
    const auto rmw_qos = qos.get_rmw_qos_profile();

    output_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);

    if (enable_time_compensation_) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_, this, false);
      tf_buffer_->setUsingDedicatedThread(true);
    }

    // Dual-source sync path
    primary_sub_.subscribe(this, primary_topic, rmw_qos);
    secondary_sub_.subscribe(this, secondary_topic, rmw_qos);

    if (sync_policy == "exact") {
      sync_exact_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(
        ExactSyncPolicy(queue_size), primary_sub_, secondary_sub_);
      sync_exact_->registerCallback(std::bind(
        &PointcloudMergeSync::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    } else {
      if (sync_policy != "approximate") {
        RCLCPP_WARN(
            this->get_logger(),
            "Unknown sync_policy='%s', fallback to approximate",
            sync_policy.c_str());
      }
      sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(queue_size), primary_sub_, secondary_sub_);
      sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(merge_time_tolerance_sec));
      sync_->registerCallback(std::bind(
        &PointcloudMergeSync::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2));
    }

    // Single-source fallback: always active, suppressed briefly when sync fires
    if (enable_fallback) {
      primary_direct_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        primary_topic, qos,
        std::bind(
          &PointcloudMergeSync::primaryDirectCallback, this,
          std::placeholders::_1));
    }

    // Initialize to well in the past so fallback is immediately active
    last_sync_stamp_ = this->now() - rclcpp::Duration::from_seconds(10.0);

    RCLCPP_INFO(
        this->get_logger(),
        "primary=%s secondary=%s output=%s tolerance=%.3fs queue=%d policy=%s fallback=%s",
        primary_topic.c_str(), secondary_topic.c_str(),
        output_topic.c_str(), merge_time_tolerance_sec, queue_size,
        sync_policy.c_str(), enable_fallback ? "true" : "false");
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

    if (enable_time_compensation_ && tf_buffer_) {
      const auto primary_stamp = rclcpp::Time(primary_msg->header.stamp);
      const auto secondary_stamp = rclcpp::Time(secondary_msg->header.stamp);
      const double abs_dt = std::abs((primary_stamp - secondary_stamp).seconds());
      if (abs_dt > max_time_compensation_sec_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Skip time compensation: dt=%.3fs exceeds max_time_compensation_sec=%.3fs",
          abs_dt, max_time_compensation_sec_);
      } else if (primary_msg->header.frame_id != odom_frame_ ||
                 secondary_msg->header.frame_id != odom_frame_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Skip time compensation: frame_id mismatch primary=%s secondary=%s expected=%s",
          primary_msg->header.frame_id.c_str(),
          secondary_msg->header.frame_id.c_str(),
          odom_frame_.c_str());
      } else {
        Eigen::Matrix4f tf;
        if (computeRelativeTransform(primary_stamp, secondary_stamp, tf)) {
          pcl::transformPointCloud(secondary_pcl, secondary_pcl, tf);
        }
      }
    }

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
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> sync_exact_;

  // Single-source fallback
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr primary_direct_sub_;

  // Timestamp of last successful sync
  rclcpp::Time last_sync_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;

  bool enable_time_compensation_{false};
  double max_time_compensation_sec_{0.2};
  std::string odom_frame_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool computeRelativeTransform(
    const rclcpp::Time & target_time,
    const rclcpp::Time & source_time,
    Eigen::Matrix4f & out)
  {
    if (!tf_buffer_) {
      return false;
    }

    geometry_msgs::msg::TransformStamped tf_target;
    geometry_msgs::msg::TransformStamped tf_source;
    try {
      tf_target = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, target_time, tf2::durationFromSec(0.0));
      tf_source = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, source_time, tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed (%s->%s): %s",
        odom_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    tf2::Transform t_target;
    tf2::Transform t_source;
    tf2::fromMsg(tf_target.transform, t_target);
    tf2::fromMsg(tf_source.transform, t_source);
    const tf2::Transform t_rel = t_target * t_source.inverse();

    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    const auto & basis = t_rel.getBasis();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        tf(r, c) = static_cast<float>(basis[r][c]);
      }
    }
    tf(0, 3) = static_cast<float>(t_rel.getOrigin().x());
    tf(1, 3) = static_cast<float>(t_rel.getOrigin().y());
    tf(2, 3) = static_cast<float>(t_rel.getOrigin().z());
    out = tf;
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudMergeSync>());
  rclcpp::shutdown();
  return 0;
}
