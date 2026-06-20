// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include <math.h>
#include <queue>

#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

double scanVoxelSize = 0.1;
double decayTime = 10.0;
double noDecayDis = 0;
double clearingDis = 30.0;
bool clearingCloud = false;
bool useSorting = false;
double quantileZ = 0.25;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double lowerBoundZ = -1.5;
double upperBoundZ = 1.0;
double disRatioZ = 0.1;
bool checkTerrainConn = true;
double terrainUnderVehicle = -0.75;
double terrainConnThre = 0.5;
double ceilingFilteringThre = 2.0;
double localTerrainMapRadius = 4.0;
double publishStampOffsetSec = 0.0;
std::string publishStampReferenceTopic = "";
double publishStampReferenceAgeAlpha = 0.2;
double publishStampReferenceAgeClampMinSec = 0.0;
double publishStampReferenceAgeClampMaxSec = 1.0;
double publishStampReferenceStampSec = 0.0;
bool publishStampReferenceStampReady = false;

rclcpp::Node::SharedPtr gNode;

// terrain voxel parameters
float terrainVoxelSize = 2.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 41;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
constexpr int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.4;
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
constexpr int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];

int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
float planarVoxelElev[kPlanarVoxelNum] = {0};
int planarVoxelConn[kPlanarVoxelNum] = {0};
std::vector<float> planarPointElev[kPlanarVoxelNum];
std::queue<int> planarVoxelQueue;

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

// state estimation callback function
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;
}

// registered laser scan callback function
void laserCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();

  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  auto fill_xyz_only = [&]() {
    pcl::PointCloud<pcl::PointXYZ> laserCloudXYZ;
    pcl::fromROSMsg(*laserCloud2, laserCloudXYZ);
    laserCloud->reserve(laserCloudXYZ.points.size());
    for (const auto & pt : laserCloudXYZ.points) {
      pcl::PointXYZI out;
      out.x = pt.x;
      out.y = pt.y;
      out.z = pt.z;
      out.intensity = 0.0f;
      laserCloud->push_back(out);
    }
  };

  const sensor_msgs::msg::PointField * intensity_field = nullptr;
  for (const auto & field : laserCloud2->fields) {
    if (field.name == "intensity") {
      intensity_field = &field;
      break;
    }
  }

  if (intensity_field == nullptr) {
    fill_xyz_only();
  } else if (intensity_field->datatype == sensor_msgs::msg::PointField::FLOAT32) {
    pcl::fromROSMsg(*laserCloud2, *laserCloud);
  } else {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*laserCloud2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*laserCloud2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*laserCloud2, "z");

    laserCloud->reserve(laserCloud2->width * laserCloud2->height);

    switch (intensity_field->datatype) {
      case sensor_msgs::msg::PointField::UINT8: {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> it_i(*laserCloud2, "intensity");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
          pcl::PointXYZI out;
          out.x = *it_x;
          out.y = *it_y;
          out.z = *it_z;
          out.intensity = static_cast<float>(*it_i);
          laserCloud->push_back(out);
        }
        break;
      }
      case sensor_msgs::msg::PointField::UINT16: {
        sensor_msgs::PointCloud2ConstIterator<uint16_t> it_i(*laserCloud2, "intensity");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
          pcl::PointXYZI out;
          out.x = *it_x;
          out.y = *it_y;
          out.z = *it_z;
          out.intensity = static_cast<float>(*it_i);
          laserCloud->push_back(out);
        }
        break;
      }
      case sensor_msgs::msg::PointField::INT8: {
        sensor_msgs::PointCloud2ConstIterator<int8_t> it_i(*laserCloud2, "intensity");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
          pcl::PointXYZI out;
          out.x = *it_x;
          out.y = *it_y;
          out.z = *it_z;
          out.intensity = static_cast<float>(*it_i);
          laserCloud->push_back(out);
        }
        break;
      }
      case sensor_msgs::msg::PointField::INT16: {
        sensor_msgs::PointCloud2ConstIterator<int16_t> it_i(*laserCloud2, "intensity");
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
          pcl::PointXYZI out;
          out.x = *it_x;
          out.y = *it_y;
          out.z = *it_z;
          out.intensity = static_cast<float>(*it_i);
          laserCloud->push_back(out);
        }
        break;
      }
      default:
        fill_xyz_only();
        break;
    }
  }

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > lowerBoundZ - disRatioZ * dis &&
        pointZ - vehicleZ < upperBoundZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// local terrain cloud callback function
void terrainCloudLocalHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudLocal2) {
  terrainCloudLocal->clear();
  pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
}

// cloud clearing callback function
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
  clearingDis = dis->data;
  clearingCloud = true;
}

void referenceCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr referenceCloud) {
  if (!gNode) {
    return;
  }

  const double nowSec = gNode->get_clock()->now().seconds();
  const double refStampSec = rclcpp::Time(referenceCloud->header.stamp).seconds();
  const double ageSec = nowSec - refStampSec;

  if (ageSec < publishStampReferenceAgeClampMinSec ||
      ageSec > publishStampReferenceAgeClampMaxSec) {
    return;
  }

  double alpha = publishStampReferenceAgeAlpha;
  if (alpha < 0.0) {
    alpha = 0.0;
  } else if (alpha > 1.0) {
    alpha = 1.0;
  }

  if (!publishStampReferenceStampReady) {
    publishStampReferenceStampSec = refStampSec;
    publishStampReferenceStampReady = true;
  } else {
    publishStampReferenceStampSec =
        (1.0 - alpha) * publishStampReferenceStampSec + alpha * refStampSec;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysisExt");
  gNode = nh;

  nh->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
  nh->declare_parameter<double>("decayTime", decayTime);
  nh->declare_parameter<double>("noDecayDis", noDecayDis);
  nh->declare_parameter<double>("clearingDis", clearingDis);
  nh->declare_parameter<bool>("useSorting", useSorting);
  nh->declare_parameter<double>("quantileZ", quantileZ);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->declare_parameter<double>("lowerBoundZ", lowerBoundZ);
  nh->declare_parameter<double>("upperBoundZ", upperBoundZ);
  nh->declare_parameter<double>("disRatioZ", disRatioZ);
  nh->declare_parameter<bool>("checkTerrainConn", checkTerrainConn);
  nh->declare_parameter<double>("terrainUnderVehicle", terrainUnderVehicle);
  nh->declare_parameter<double>("terrainConnThre", terrainConnThre);
  nh->declare_parameter<double>("ceilingFilteringThre", ceilingFilteringThre);
  nh->declare_parameter<double>("localTerrainMapRadius", localTerrainMapRadius);
  nh->declare_parameter<double>("publishStampOffsetSec", publishStampOffsetSec);
  nh->declare_parameter<std::string>("publishStampReferenceTopic", publishStampReferenceTopic);
  nh->declare_parameter<double>("publishStampReferenceAgeAlpha", publishStampReferenceAgeAlpha);
  nh->declare_parameter<double>("publishStampReferenceAgeClampMinSec", publishStampReferenceAgeClampMinSec);
  nh->declare_parameter<double>("publishStampReferenceAgeClampMaxSec", publishStampReferenceAgeClampMaxSec);

  nh->get_parameter("scanVoxelSize", scanVoxelSize);
  nh->get_parameter("decayTime", decayTime);
  nh->get_parameter("noDecayDis", noDecayDis);
  nh->get_parameter("clearingDis", clearingDis);
  nh->get_parameter("useSorting", useSorting);
  nh->get_parameter("quantileZ", quantileZ);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->get_parameter("lowerBoundZ", lowerBoundZ);
  nh->get_parameter("upperBoundZ", upperBoundZ);
  nh->get_parameter("disRatioZ", disRatioZ);
  nh->get_parameter("checkTerrainConn", checkTerrainConn);
  nh->get_parameter("terrainUnderVehicle", terrainUnderVehicle);
  nh->get_parameter("terrainConnThre", terrainConnThre);
  nh->get_parameter("ceilingFilteringThre", ceilingFilteringThre);
  nh->get_parameter("localTerrainMapRadius", localTerrainMapRadius);
  nh->get_parameter("publishStampOffsetSec", publishStampOffsetSec);
  nh->get_parameter("publishStampReferenceTopic", publishStampReferenceTopic);
  nh->get_parameter("publishStampReferenceAgeAlpha", publishStampReferenceAgeAlpha);
  nh->get_parameter("publishStampReferenceAgeClampMinSec", publishStampReferenceAgeClampMinSec);
  nh->get_parameter("publishStampReferenceAgeClampMaxSec", publishStampReferenceAgeClampMaxSec);

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
      "lidar_odometry", sensor_qos, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", sensor_qos, laserCloudHandler);

  auto subClearing = nh->create_subscription<std_msgs::msg::Float32>(
      "cloud_clearing", 5, clearingHandler);

  auto subTerrainCloudLocal =
      nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        "terrain_map", sensor_qos, terrainCloudLocalHandler);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subReferenceCloud;
  if (!publishStampReferenceTopic.empty()) {
    subReferenceCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      publishStampReferenceTopic, sensor_qos, referenceCloudHandler);
  }

  auto pubTerrainCloud =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_ext", 2);

  for (int i = 0; i < kTerrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (newlaserCloud) {
      newlaserCloud = false;

      // terrain voxel roll over
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop->points[i];

        int indX =
            static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) /
                             terrainVoxelSize) +
            terrainVoxelHalfWidth;
        int indY =
            static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) /
                             terrainVoxelSize) +
            terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }

      for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[ind];

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++) {
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity <
                     decayTime ||
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud)) {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 10;
           indX <= terrainVoxelHalfWidth + 10; indX++) {
        for (int indY = terrainVoxelHalfWidth - 10;
             indY <= terrainVoxelHalfWidth + 10; indY++) {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // estimate ground and compute elevation for each point
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        planarVoxelElev[i] = 0;
        planarVoxelConn[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {
          int indX =
              static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;
          int indY =
              static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                    .push_back(point.z);
              }
            }
          }
        }
      }

      if (useSorting) {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = static_cast<int>(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            planarVoxelElev[i] = planarPointElev[i][quantileID];
          }
        }
      } else {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      // check terrain connectivity to remove ceiling
      if (checkTerrainConn) {
        int ind =
            planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
        if (planarPointElev[ind].size() == 0)
          planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;

        planarVoxelQueue.push(ind);
        planarVoxelConn[ind] = 1;
        while (!planarVoxelQueue.empty()) {
          int front = planarVoxelQueue.front();
          planarVoxelConn[front] = 2;
          planarVoxelQueue.pop();

          int indX = static_cast<int>(front / planarVoxelWidth);
          int indY = front % planarVoxelWidth;
          for (int dX = -10; dX <= 10; dX++) {
            for (int dY = -10; dY <= 10; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                ind = planarVoxelWidth * (indX + dX) + indY + dY;
                if (planarVoxelConn[ind] == 0 &&
                    planarPointElev[ind].size() > 0) {
                  if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) <
                      terrainConnThre) {
                    planarVoxelQueue.push(ind);
                    planarVoxelConn[ind] = 1;
                  } else if (fabs(planarVoxelElev[front] -
                                  planarVoxelElev[ind]) >
                             ceilingFilteringThre) {
                    planarVoxelConn[ind] = -1;
                  }
                }
              }
            }
          }
        }
      }

      // compute terrain map beyond localTerrainMapRadius
      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
            dis > localTerrainMapRadius) {
          int indX =
              static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;
          int indY =
              static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) /
                               planarVoxelSize) +
              planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            int ind = planarVoxelWidth * indX + indY;
            float disZ = fabs(point.z - planarVoxelElev[ind]);
            if (disZ < vehicleHeight &&
                (planarVoxelConn[ind] == 2 || !checkTerrainConn)) {
              terrainCloudElev->push_back(point);
              terrainCloudElev->points[terrainCloudElevSize].x = point.x;
              terrainCloudElev->points[terrainCloudElevSize].y = point.y;
              terrainCloudElev->points[terrainCloudElevSize].z = point.z;
              terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

              terrainCloudElevSize++;
            }
          }
        }
      }

      // merge in local terrain map within localTerrainMapRadius
      int terrainCloudLocalSize = terrainCloudLocal->points.size();
      for (int i = 0; i < terrainCloudLocalSize; i++) {
        point = terrainCloudLocal->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (dis <= localTerrainMapRadius) {
          terrainCloudElev->push_back(point);
        }
      }

      clearingCloud = false;

      // publish points with elevation
      sensor_msgs::msg::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      // 可配置时间戳偏移：用于对齐多源链路在 costmap 的时间窗口。
      double publishStampSec = std::max(0.0, laserCloudTime + publishStampOffsetSec);
      if (publishStampReferenceStampReady) {
        const double nowSec = nh->get_clock()->now().seconds();
        const double refAgeSec = nowSec - publishStampReferenceStampSec;
        if (refAgeSec >= publishStampReferenceAgeClampMinSec &&
            refAgeSec <= publishStampReferenceAgeClampMaxSec &&
            publishStampReferenceStampSec > 0.0) {
          publishStampSec = publishStampReferenceStampSec;
        }
      }
      publishStampSec = std::max(0.0, publishStampSec + publishStampOffsetSec);
      terrainCloud2.header.stamp =
          rclcpp::Time(static_cast<uint64_t>(publishStampSec * 1e9));
      terrainCloud2.header.frame_id = "odom";
      pubTerrainCloud->publish(terrainCloud2);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
