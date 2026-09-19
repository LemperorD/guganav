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

  // 平地上没有障碍：带内的障碍云为空，但回波云要保留地面回波——清除射线正是
  // 靠这些回波判定"该方向的路径为空"。
  TEST_F(TerrainAnalysisTest, Run_FlatGround_ReturnsKeepGroundEchoes) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundCloud(21, 0.1, 0.01);
    sendCloud(cloud, 100.0);

    EXPECT_TRUE(terrain_->frameObstacleCloud().points.empty())
        << "地面点落在输出带死区内，不应进入障碍云";
    EXPECT_GT(terrain_->frameReturnCloud().points.size(), 0U)
        << "地面回波应保留在回波云中";
  }

  // 障碍点在带内：两份当帧输出都含它，且 intensity 为离地高度。
  TEST_F(TerrainAnalysisTest, Run_ObstacleInBand_FrameOutputsCarryHeight) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundAndObstacleCloud(21, 0.1, 0.0, 0.06);
    sendCloud(cloud, 100.0);

    EXPECT_GT(terrain_->frameObstacleCloud().points.size(), 0U);

    float max_intensity = 0;
    for (const auto& p : terrain_->frameObstacleCloud().points) {
      max_intensity = std::max(max_intensity, p.intensity);
    }
    EXPECT_GT(max_intensity, 0.05F);
    EXPECT_GT(terrain_->frameReturnCloud().points.size(),
              terrain_->frameObstacleCloud().points.size())
        << "回波云还应含输出带之外的地面回波";
  }

  // 远低于地面地板的点两份都不输出：它既不是障碍，也不该给出清除射线。
  TEST_F(TerrainAnalysisTest,
         Run_PointBelowGroundFloor_ExcludedFromFrameOutputs) {
    sendOdom(0, 0, 0);

    auto cloud = MakeGroundCloud(21, 0.1, 0.01);
    pcl::PointXYZI below_floor{2.0F, 2.0F, -1.0F, 0};
    cloud->push_back(below_floor);
    sendCloud(cloud, 100.0);

    const auto has_point_near = [](const auto& points, float x, float y) {
      for (const auto& p : points) {
        if (std::abs(p.x - x) < 1e-3F && std::abs(p.y - y) < 1e-3F) {
          return true;
        }
      }
      return false;
    };
    EXPECT_FALSE(
        has_point_near(terrain_->frameObstacleCloud().points, 2.0F, 2.0F));
    EXPECT_FALSE(
        has_point_near(terrain_->frameReturnCloud().points, 2.0F, 2.0F))
        << "低于地面地板的点不应进入回波云";
  }

  // 当帧输出只反映本帧：上一帧的障碍点留在累计输出里，但不进入本帧两份输出。
  TEST_F(TerrainAnalysisTest, Run_FrameOutputsUseCurrentFrameOnly) {
    sendOdom(0, 0, 0);

    sendCloud(MakeGroundAndObstacleCloud(21, 0.1, 0.01, 0.06), 100.0);
    EXPECT_GT(terrain_->frameObstacleCloud().points.size(), 0U);

    // 第二帧里该障碍已消失（时间间隔仍在衰减阈值内）。
    sendCloud(MakeGroundCloud(21, 0.1, 0.01), 100.05);

    EXPECT_TRUE(terrain_->frameObstacleCloud().points.empty())
        << "本帧没有障碍时，当帧障碍云应为空";
    EXPECT_FALSE(terrain_->obstacleCloud().points.empty())
        << "该障碍仍在累计云的衰减窗口内";
  }

  // 清除端点云每个方位角桶一个端点：桶内有回波就用最远的那个（哪怕它在平面网格
  // 之外），桶内没有回波就合成一个远端端点——那个方位的光束一路没有碰到东西，
  // 也就是那里为空；不补端点，该方位的旧标记永远清不掉。障碍云需要离地高度，
  // 因此不含网格外的点。
  TEST_F(TerrainAnalysisTest, Run_ReturnFan_CoversEveryAzimuth) {
    sendOdom(0, 0, 0);

    // 只在 +x 方位放回波，最远 8 m（超出平面网格的 ±5.1 m）
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (int i = 0; i <= 10; ++i) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(6.0 + 0.2 * static_cast<double>(i));
      p.y = 0.0F;
      p.z = 0.01F;
      p.intensity = 0;
      cloud->push_back(p);
    }
    sendCloud(cloud, 100.0);

    const auto& fan = terrain_->frameReturnCloud();
    EXPECT_EQ(fan.points.size(), 720U) << "0.5° 一桶，每个方位角都要有端点";

    // 按同样的分桶规则取 +x 桶（下标 360）与反方向桶（下标 0）的距离
    const auto distance_at_bin = [&](int wanted) {
      for (const auto& p : fan.points) {
        const double angle = std::atan2(p.y, p.x);
        const int bin = static_cast<int>(
            std::floor((angle + M_PI) / (2.0 * M_PI) * 720.0));
        if (bin == wanted) {
          return std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
        }
      }
      return -1.0;
    };
    EXPECT_NEAR(distance_at_bin(360), 8.0, 0.1) << "+x 方位应取该桶最远的回波";
    EXPECT_NEAR(distance_at_bin(0), 12.0, 0.1) << "没有回波的方位应补远端端点";

    EXPECT_TRUE(terrain_->frameObstacleCloud().points.empty())
        << "网格外没有地面估计，不该输出障碍点";
  }
}  // namespace terrain_analysis