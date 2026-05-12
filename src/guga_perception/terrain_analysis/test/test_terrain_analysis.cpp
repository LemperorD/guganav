#include "terrain_analysis/terrain_analysis.hpp"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

class TerrainAnalysisTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    nh_ = rclcpp::Node::make_shared("test_terrain_analysis");

    // Declare + get params (same as main.cpp)
    nh_->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
    nh_->declare_parameter<double>("decayTime", decayTime);
    nh_->declare_parameter<double>("noDecayDis", noDecayDis);
    nh_->declare_parameter<double>("clearingDis", clearingDis);
    nh_->declare_parameter<bool>("useSorting", useSorting);
    nh_->declare_parameter<double>("quantileZ", quantileZ);
    nh_->declare_parameter<bool>("considerDrop", considerDrop);
    nh_->declare_parameter<bool>("limitGroundLift", limitGroundLift);
    nh_->declare_parameter<double>("maxGroundLift", maxGroundLift);
    nh_->declare_parameter<bool>("clearDyObs", clearDyObs);
    nh_->declare_parameter<double>("minDyObsDis", minDyObsDis);
    nh_->declare_parameter<double>("minDyObsAngle", minDyObsAngle);
    nh_->declare_parameter<double>("minDyObsRelZ", minDyObsRelZ);
    nh_->declare_parameter<double>("absDyObsRelZThre", absDyObsRelZThre);
    nh_->declare_parameter<double>("minDyObsVFOV", minDyObsVFOV);
    nh_->declare_parameter<double>("maxDyObsVFOV", maxDyObsVFOV);
    nh_->declare_parameter<int>("minDyObsPointNum", minDyObsPointNum);
    nh_->declare_parameter<bool>("noDataObstacle", noDataObstacle);
    nh_->declare_parameter<int>("noDataBlockSkipNum", noDataBlockSkipNum);
    nh_->declare_parameter<int>("minBlockPointNum", minBlockPointNum);
    nh_->declare_parameter<double>("vehicleHeight", vehicleHeight);
    nh_->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
    nh_->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
    nh_->declare_parameter<double>("minRelZ", minRelZ);
    nh_->declare_parameter<double>("maxRelZ", maxRelZ);
    nh_->declare_parameter<double>("disRatioZ", disRatioZ);

    nh_->get_parameter("scanVoxelSize", scanVoxelSize);
    nh_->get_parameter("decayTime", decayTime);
    nh_->get_parameter("noDecayDis", noDecayDis);
    nh_->get_parameter("clearingDis", clearingDis);
    nh_->get_parameter("useSorting", useSorting);
    nh_->get_parameter("quantileZ", quantileZ);
    nh_->get_parameter("considerDrop", considerDrop);
    nh_->get_parameter("limitGroundLift", limitGroundLift);
    nh_->get_parameter("maxGroundLift", maxGroundLift);
    nh_->get_parameter("clearDyObs", clearDyObs);
    nh_->get_parameter("minDyObsDis", minDyObsDis);
    nh_->get_parameter("minDyObsAngle", minDyObsAngle);
    nh_->get_parameter("minDyObsRelZ", minDyObsRelZ);
    nh_->get_parameter("absDyObsRelZThre", absDyObsRelZThre);
    nh_->get_parameter("minDyObsVFOV", minDyObsVFOV);
    nh_->get_parameter("maxDyObsVFOV", maxDyObsVFOV);
    nh_->get_parameter("minDyObsPointNum", minDyObsPointNum);
    nh_->get_parameter("noDataObstacle", noDataObstacle);
    nh_->get_parameter("noDataBlockSkipNum", noDataBlockSkipNum);
    nh_->get_parameter("minBlockPointNum", minBlockPointNum);
    nh_->get_parameter("vehicleHeight", vehicleHeight);
    nh_->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
    nh_->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
    nh_->get_parameter("minRelZ", minRelZ);
    nh_->get_parameter("maxRelZ", maxRelZ);
    nh_->get_parameter("disRatioZ", disRatioZ);

    // Duplicate params discovery for clean test each time
    for (int i = 0; i < kTerrainVoxelNum; i++) {
      terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      terrainVoxelUpdateNum[i] = 0;
      terrainVoxelUpdateTime[i] = 0;
    }
    for (int i = 0; i < kPlanarVoxelNum; i++) {
      planarVoxelElev[i] = 0;
      planarVoxelEdge[i] = 0;
      planarVoxelDyObs[i] = 0;
      planarPointElev[i].clear();
    }

    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

    newlaserCloud = false;
    clearingCloud = false;
    systemInited = false;
    systemInitTime = 0;
    noDataInited = 0;
  }

  void TearDown() override {
    nh_.reset();
    rclcpp::shutdown();
  }

  void SendOdom(double x, double y, double z, double yaw) {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = z;
    // set yaw quaternion
    msg->pose.pose.orientation.x = 0;
    msg->pose.pose.orientation.y = 0;
    msg->pose.pose.orientation.z = sin(yaw / 2.0);
    msg->pose.pose.orientation.w = cos(yaw / 2.0);
    odometryHandler(msg);
  }

  void SendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 double timestamp_sec) {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud, *msg);
    msg->header.stamp =
        rclcpp::Time(static_cast<uint64_t>(timestamp_sec * 1e9));
    laserCloudHandler(msg);
  }

  rclcpp::Node::SharedPtr nh_;
};

TEST_F(TerrainAnalysisTest, FlatGround_DetectsGround_ZNearZero) {
  // Vehicle at origin, flat
  SendOdom(0, 0, 0, 0);

  // Flat ground at z=0 in vehicle frame, spread points around center
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < 500; i++) {
    pcl::PointXYZI p;
    p.x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0f;
    p.y = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0f;
    p.z = 0.01f;
    p.intensity = 0;
    cloud->push_back(p);
  }
  SendCloud(cloud, 100.0);
  processTerrainData();

  // terrainCloudElev should have points after processing
  EXPECT_GT(terrainCloudElev->points.size(), 0u)
      << "terrainCloudElev should contain output points";

  // Ground should be near 0, so intensity (height above ground) near 0
  float max_intensity = 0;
  for (const auto& p : terrainCloudElev->points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_LT(max_intensity, 0.5f)
      << "Flat ground: all intensities should be small";
}

TEST_F(TerrainAnalysisTest, Obstacle_DetectedAboveGround) {
  SendOdom(0, 0, 0.5, 0);

  // Ground points at vehicle height + many elevated points
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < 2000; i++) {
    pcl::PointXYZI p;
    p.x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 3.0f;
    p.y = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 3.0f;
    p.z = 0.5f;  // at vehicle height → relZ=0, ground
    p.intensity = 0;
    cloud->push_back(p);
  }
  // Patch of elevated points (relZ = 0.15, within maxRelZ)
  for (int i = 0; i < 100; i++) {
    pcl::PointXYZI p;
    p.x = 0.2f + (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.1f;
    p.y = 0.2f + (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.1f;
    p.z = 0.65f;  // 0.15 above vehicle
    p.intensity = 0;
    cloud->push_back(p);
  }

  SendCloud(cloud, 100.0);
  processTerrainData();

  EXPECT_GT(terrainCloudElev->points.size(), 0u);

  float max_intensity = 0;
  for (const auto& p : terrainCloudElev->points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_GT(max_intensity, 0.05f)
      << "Elevated points should produce non-zero intensity";
}
