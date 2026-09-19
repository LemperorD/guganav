#include "terrain_analysis/terrain_analysis_node.hpp"
#include "terrain_analysis/core/persistent_voxel_map.hpp"
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

    /** @brief 记录本帧的雷达位置（相当于里程计回调写入的那一份）。 */
    void sendOdom(double x, double y, double z) {
      lidar_position_ = {x, y, z};
    }

    void sendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                   double timestamp_sec) {
      terrain_->voxelMap().ingest(*cloud, lidar_position_, timestamp_sec);
    }

    std::unique_ptr<TerrainAnalysis> terrain_;
    guga_common::Point3d lidar_position_;
  };

  // 纯平面地面点云经过全管线后不输出障碍点：地面落在
  // `min_obstacle_height` 的死区内（此前输出过 intensity≈0 的地面点，
  // 由下游的 intensity 门限再滤掉；现在在 terrain 内就不再输出）。
  TEST_F(TerrainAnalysisTest, Run_FlatGround_NoObstacleOutput) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundCloud(21, 0.1, 0.01);
    sendCloud(cloud, 100.0);
    terrain_->processOnce();

    EXPECT_TRUE(terrain_->obstacleCloud().points.empty())
        << "平坦地面不应输出障碍点";
  }

  // 地面上方有障碍点时，输出点云包含非零离地高度。
  // 障碍高度需低于 ceilingClearance（默认 0.62 m，距局部地面）——达到该值的点
  // 会被当作可从下方通过的悬空结构而不输出；此处取 0.06，位于输出带内。
  TEST_F(TerrainAnalysisTest, Run_ObstacleAboveGround_OutputsNonZeroIntensity) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundAndObstacleCloud(21, 0.1, 0.0, 0.06);
    sendCloud(cloud, 100.0);
    terrain_->processOnce();

    EXPECT_GT(terrain_->obstacleCloud().points.size(), 0U);

    float max_intensity = 0;
    for (const auto& p : terrain_->obstacleCloud().points) {
      max_intensity = std::max(max_intensity, p.intensity);
    }
    EXPECT_GT(max_intensity, 0.05F) << "抬高到输出带内的点应产生非零离地高度";
  }

  // 孤立障碍点所在 voxel 点数不足时被过滤，不出现在输出中
  TEST_F(TerrainAnalysisTest,
         Run_IsolatedObstacleInSparseVoxel_ExcludedFromOutput) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundCloud(11, 0.1, 0.0);
    pcl::PointXYZI obs{3.0F, 3.0F, 0.3F, 0};
    cloud->push_back(obs);
    sendCloud(cloud, 100.0);
    terrain_->processOnce();

    bool found_isolated = false;
    for (const auto& p : terrain_->obstacleCloud().points) {
      if (p.x > 2.5F && p.intensity > 0.1F) {
        found_isolated = true;
        break;
      }
    }
    EXPECT_FALSE(found_isolated) << "稀疏体素里的孤立障碍点应被排除";
  }
}  // namespace terrain_analysis