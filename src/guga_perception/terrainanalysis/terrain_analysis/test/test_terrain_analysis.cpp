#include "terrain_analysis/terrain_analysis.hpp"
#include "terrain_analysis/core/algorithm.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <random>

class TerrainAnalysisTest : public testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_terrain_analysis");
    terrain_ = std::make_unique<TerrainAnalysis>(node_.get());
  }

  void TearDown() override {
    terrain_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void sendOdom(double x, double y, double z, double yaw) {
    terrain_->context_.onOdometry(x, y, z, 0.0, 0.0, yaw);
  }

  void sendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                 double timestamp_sec) {
    terrain_->context_.onLaserCloud(cloud, timestamp_sec);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<TerrainAnalysis> terrain_;
};

// ── Pipeline ──
TEST_F(TerrainAnalysisTest, Run_FlatGround_OutputsLowIntensity) {
  sendOdom(0, 0, 0, 0);

  std::mt19937 rng{42};  // fixed seed for reproducibility
  std::uniform_real_distribution<float> dist(-1.0F, 1.0F);
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < 500; i++) {
    cloud->push_back({dist(rng), dist(rng), 0.01F, 0});
  }
  sendCloud(cloud, 100.0);
  TerrainAlgorithm::run(terrain_->context_.cfg, terrain_->context_.state);

  EXPECT_GT(terrain_->context_.state.terrainCloudElev().points.size(), 0U);

  float max_intensity = 0;
  for (const auto& p : terrain_->context_.state.terrainCloudElev().points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_LT(max_intensity, 0.5F) << "Flat ground should produce small heights";
}

TEST_F(TerrainAnalysisTest, Run_ObstacleAboveGround_OutputsNonZeroIntensity) {
  sendOdom(0, 0, 0, 0);

  auto cloud = MakeGroundAndObstacleCloud(21, 0.1, 0.0, 0.15);
  sendCloud(cloud, 100.0);
  TerrainAlgorithm::run(terrain_->context_.cfg, terrain_->context_.state);

  EXPECT_GT(terrain_->context_.state.terrainCloudElev().points.size(), 0U);

  float max_intensity = 0;
  for (const auto& p : terrain_->context_.state.terrainCloudElev().points) {
    max_intensity = std::max(max_intensity, p.intensity);
  }
  EXPECT_GT(max_intensity, 0.05F)
      << "Elevated points should produce non-zero height";
}

TEST_F(TerrainAnalysisTest,
       Run_IsolatedObstacleInSparseVoxel_ExcludedFromOutput) {
  sendOdom(0, 0, 0, 0);

  // ground cluster near origin, obstacle far away in isolated voxel
  auto cloud = MakeGroundCloud(11, 0.1, 0.0);
  pcl::PointXYZI obs{3.0F, 3.0F, 0.3F, 0};
  cloud->push_back(obs);
  sendCloud(cloud, 100.0);
  TerrainAlgorithm::run(terrain_->context_.cfg, terrain_->context_.state);

  // isolated point has sparse voxel (< min_block_point_num), excluded
  bool found_isolated = false;
  for (const auto& p : terrain_->context_.state.terrainCloudElev().points) {
    if (p.x > 2.5F && p.intensity > 0.1F) {
      found_isolated = true;
      break;
    }
  }
  EXPECT_FALSE(found_isolated)
      << "Isolated obstacle in sparse voxel should be excluded";
}
