#include "terrain_analysis/terrain_analysis_node.hpp"
#include "terrain_analysis/core/algorithm.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>

namespace terrain_analysis {
  class TerrainAnalysisTest : public testing::Test {
  protected:
    void SetUp() override {
      rclcpp::init(0, nullptr);
      // TerrainAnalysis 已组件化（rclcpp::Node 子类），直接以 NodeOptions 构造
      terrain_ = std::make_unique<TerrainAnalysis>(rclcpp::NodeOptions());
    }

    void TearDown() override {
      terrain_.reset();
      rclcpp::shutdown();
    }

    void sendOdom(double x, double y, double z, double yaw) {
      terrain_analysis::algorithm::ingestOdometry(
          terrain_->config(), terrain_->state(), x, y, z, 0.0, 0.0, yaw);
    }

    void sendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                   double timestamp_sec) {
      terrain_analysis::algorithm::ingestLaserCloud(
          terrain_->config(), terrain_->state(), cloud, timestamp_sec);
    }

    std::unique_ptr<TerrainAnalysis> terrain_;
  };

  // 平面地面点云经过全管线后，输出的 intensity（离地高度）都接近零
  TEST_F(TerrainAnalysisTest, Run_FlatGround_OutputsLowIntensity) {
    sendOdom(0, 0, 0, 0);

    auto cloud = MakeGroundCloud(21, 0.1, 0.01);
    sendCloud(cloud, 100.0);
    terrain_analysis::algorithm::run(terrain_->config(), terrain_->state());

    EXPECT_GT(terrain_->state().terrain_cloud_elev->points.size(), 0U);

    float max_intensity = 0;
    for (const auto& p : terrain_->state().terrain_cloud_elev->points) {
      max_intensity = std::max(max_intensity, p.intensity);
    }
    EXPECT_LT(max_intensity, 0.5F)
        << "Flat ground should produce small heights";
  }

  // 地面上方有障碍点时，输出点云包含非零离地高度
  TEST_F(TerrainAnalysisTest, Run_ObstacleAboveGround_OutputsNonZeroIntensity) {
    sendOdom(0, 0, 0, 0);

    auto cloud = MakeGroundAndObstacleCloud(21, 0.1, 0.0, 0.15);
    sendCloud(cloud, 100.0);
    terrain_analysis::algorithm::run(terrain_->config(), terrain_->state());

    EXPECT_GT(terrain_->state().terrain_cloud_elev->points.size(), 0U);

    float max_intensity = 0;
    for (const auto& p : terrain_->state().terrain_cloud_elev->points) {
      max_intensity = std::max(max_intensity, p.intensity);
    }
    EXPECT_GT(max_intensity, 0.05F)
        << "Elevated points should produce non-zero height";
  }

  // 孤立障碍点所在 voxel 点数不足时被过滤，不出现在输出中
  TEST_F(TerrainAnalysisTest,
         Run_IsolatedObstacleInSparseVoxel_ExcludedFromOutput) {
    sendOdom(0, 0, 0, 0);

    auto cloud = MakeGroundCloud(11, 0.1, 0.0);
    pcl::PointXYZI obs{3.0F, 3.0F, 0.3F, 0};
    cloud->push_back(obs);
    sendCloud(cloud, 100.0);
    terrain_analysis::algorithm::run(terrain_->config(), terrain_->state());

    bool found_isolated = false;
    for (const auto& p : terrain_->state().terrain_cloud_elev->points) {
      if (p.x > 2.5F && p.intensity > 0.1F) {
        found_isolated = true;
        break;
      }
    }
    EXPECT_FALSE(found_isolated)
        << "Isolated obstacle in sparse voxel should be excluded";
  }
}  // namespace terrain_analysis