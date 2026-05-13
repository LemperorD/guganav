#include "terrain_analysis/terrain_analysis.hpp"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>

class TerrainAnalysisTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    nh_ = rclcpp::Node::make_shared("test_terrain_analysis");
    terrain_ = std::make_unique<TerrainAnalysis>(nh_.get());
  }

  void TearDown() override {
    terrain_.reset();
    nh_.reset();
    rclcpp::shutdown();
  }

  void SendOdom(double x, double y, double z, double yaw) {
    terrain_->ctx_.onOdometry(x, y, z, 0.0, 0.0, yaw);
  }

  void SendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 double timestamp_sec) {
    terrain_->ctx_.onLaserCloud(cloud, timestamp_sec);
  }

  rclcpp::Node::SharedPtr nh_;
  std::unique_ptr<TerrainAnalysis> terrain_;
};

TEST_F(TerrainAnalysisTest, FlatGround_DetectsGround_ZNearZero) {
  SendOdom(0, 0, 0, 0);

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
  terrain_->ctx_.processTerrainData();

  EXPECT_GT(terrain_->terrainCloudElev().points.size(), 0u)
      << "terrainCloudElev should contain output points";

  float max_intensity = 0;
  for (const auto& p : terrain_->terrainCloudElev().points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_LT(max_intensity, 0.5f)
      << "Flat ground: all intensities should be small";
}

TEST_F(TerrainAnalysisTest, Obstacle_DetectedAboveGround) {
  SendOdom(0, 0, 0, 0);

  // Fixed grid: 20x20 = 400 ground + 400 elevated, covering 2m x 2m
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -10; ix <= 10; ix++) {
    for (int iy = -10; iy <= 10; iy++) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * 0.1f;
      p.y = static_cast<float>(iy) * 0.1f;
      p.z = 0.0f;
      p.intensity = 0;
      cloud->push_back(p);
      p.z = 0.15f;
      cloud->push_back(p);
    }
  }

  SendCloud(cloud, 100.0);
  terrain_->ctx_.processTerrainData();

  EXPECT_GT(terrain_->terrainCloudElev().points.size(), 0u);

  float max_intensity = 0;
  for (const auto& p : terrain_->terrainCloudElev().points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_GT(max_intensity, 0.05f)
      << "Elevated points should produce non-zero intensity";
}

TEST_F(TerrainAnalysisTest, IsolatedObstacle_RejectedWhenVoxelSparse) {
  SendOdom(0, 0, 0, 0);

  // Ground cluster in a tight region near origin, obstacle far away
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -5; ix <= 5; ix++) {
    for (int iy = -5; iy <= 5; iy++) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(ix) * 0.1f;
      p.y = static_cast<float>(iy) * 0.1f;
      p.z = 0.0f;
      p.intensity = 0;
      cloud->push_back(p);
    }
  }
  // Single obstacle far outside the ground cluster, isolated voxel
  pcl::PointXYZI obs;
  obs.x = 3.0f;
  obs.y = 3.0f;
  obs.z = 0.3f;
  obs.intensity = 0;
  cloud->push_back(obs);

  SendCloud(cloud, 100.0);
  terrain_->ctx_.processTerrainData();

  // The isolated obstacle point is in a voxel with < minBlockPointNum (10)
  // ground points, so it should be excluded from output
  bool found_isolated = false;
  for (const auto& p : terrain_->terrainCloudElev().points) {
    if (p.x > 2.5f && p.intensity > 0.1f) {
      found_isolated = true;
      break;
    }
  }
  EXPECT_FALSE(found_isolated)
      << "Isolated obstacle in sparse voxel should be excluded";
}
