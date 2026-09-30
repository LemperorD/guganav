#include "terrain_analysis/terrain_analysis_node.hpp"
#include "terrain_analysis/persistent_voxel_map.hpp"
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

    /**
     * @brief 同步跑一帧：节点的公有入口，不必经 ROS
     * 话题，也就不需要任何内部访问。
     */
    void sendCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                   double timestamp_sec) {
      terrain_->processFrame(*cloud, lidar_position_, timestamp_sec);
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

    bool found_isolated = false;
    for (const auto& p : terrain_->obstacleCloud().points) {
      if (p.x > 2.5F && p.intensity > 0.1F) {
        found_isolated = true;
        break;
      }
    }
    EXPECT_FALSE(found_isolated) << "稀疏体素里的孤立障碍点应被排除";
  }

  // 平地上没有障碍：返回的雷达射线云要保留地面返回的雷达射线——清除射线正是
  // 靠这些返回的雷达射线判定"该方向的路径为空"。
  TEST_F(TerrainAnalysisTest, Run_FlatGround_ReturnsKeepGroundEchoes) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundCloud(21, 0.1, 0.01);
    sendCloud(cloud, 100.0);

    EXPECT_GT(terrain_->frameReturnCloud().points.size(), 0U)
        << "地面返回的雷达射线应保留在返回的雷达射线云中";
  }

  // 障碍点也在当帧返回的雷达射线云里：清除端点不止来自地面。
  TEST_F(TerrainAnalysisTest, Run_ObstacleInBand_IncludedInFrameReturns) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundAndObstacleCloud(21, 0.1, 0.0, 0.06);
    sendCloud(cloud, 100.0);

    int raised_points = 0;
    for (const auto& p : terrain_->frameReturnCloud().points) {
      if (p.z > 0.05F) {
        raised_points++;
      }
    }
    EXPECT_GT(raised_points, 0) << "抬高到带内的点也应作为清除端点保留";
  }

  // 远低于地面地板的点不进入返回的雷达射线云：它既不是障碍，也不该给出清除射线。
  TEST_F(TerrainAnalysisTest,
         Run_PointBelowGroundFloor_ExcludedFromFrameReturns) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundCloud(21, 0.1, 0.01);
    pcl::PointXYZI below_floor{2.0F, 2.0F, -1.0F, 0};
    cloud->push_back(below_floor);
    sendCloud(cloud, 100.0);

    for (const auto& p : terrain_->frameReturnCloud().points) {
      EXPECT_FALSE(std::abs(p.x - 2.0F) < 1e-3F && std::abs(p.y - 2.0F) < 1e-3F)
          << "低于地面地板的点不应进入返回的雷达射线云";
    }
  }

  // 当帧只反映本帧：上一帧的障碍点留在累计输出里，但不进入本帧返回的雷达射线云。
  TEST_F(TerrainAnalysisTest, Run_FrameReturnsUseCurrentFrameOnly) {
    sendOdom(0, 0, 0);

    const auto raised_point_count = [](const auto& points) {
      int count = 0;
      for (const auto& p : points) {
        if (p.z > 0.05F) {
          count++;
        }
      }
      return count;
    };

    sendCloud(MakeGroundAndObstacleCloud(21, 0.1, 0.01, 0.06), 100.0);
    EXPECT_GT(raised_point_count(terrain_->frameReturnCloud().points), 0);

    // 第二帧里该障碍已消失（时间间隔仍在衰减阈值内）。
    sendCloud(MakeGroundCloud(21, 0.1, 0.01), 100.05);

    EXPECT_EQ(raised_point_count(terrain_->frameReturnCloud().points), 0)
        << "本帧没有障碍时，当帧返回的雷达射线云不应再含它";
    EXPECT_FALSE(terrain_->obstacleCloud().points.empty())
        << "该障碍仍在累计云的衰减窗口内";
  }

  // 网格外的返回的雷达射线仍要进入清除云：它同样证明该方向的路径为空，代价地图层会把端点
  // 裁剪到自己的边界再画射线；丢掉它，该方向就一条射线都没有。本云没有高度语义，
  // intensity 必须为 0（输入云借它携带的观测时刻不能外泄）。
  TEST_F(TerrainAnalysisTest, Run_ReturnOutsideGrid_StillUsedForClearing) {
    sendOdom(0, 0, 0);

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (double x = 6.0; x <= 8.0; x += 0.2) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(x);
      p.y = 0.0F;
      p.z = 0.01F;
      p.intensity = 0;
      cloud->push_back(p);
    }
    sendCloud(cloud, 100.0);

    EXPECT_EQ(terrain_->frameReturnCloud().points.size(), cloud->size())
        << "网格外的返回的雷达射线应全部进入清除云";
    for (const auto& p : terrain_->frameReturnCloud().points) {
      EXPECT_FLOAT_EQ(p.intensity, 0.0F)
          << "本云没有高度语义，intensity 恒为 0";
    }
  }
}  // namespace terrain_analysis