#include "terrain_analysis/core/persistent_voxel_map.hpp"
#include "gtest/gtest.h"
#include "test_helpers.hpp"

#include <cmath>

namespace terrain_analysis {
  /** @brief 前半段（PersistentVoxelMap）的帧输入接收与裁剪。 */
  class FrameIngestTest : public testing::Test {
  protected:
    PersistentVoxelMap voxel_map_;

    /** @brief 首帧时刻是内部字段，测试体经 fixture 成员读取。 */
    double initTime() const {
      return voxel_map_.init_time_;
    }
  };

  // 默认构造后没有待处理帧，也没有记下任何雷达位置
  TEST_F(FrameIngestTest, DefaultState_NoPendingFrame) {
    EXPECT_FALSE(voxel_map_.hasPendingFrame());
    EXPECT_TRUE(voxel_map_.frameCloud().points.empty());
  }

  // 接一帧后记下雷达位置、帧时刻，并标记有帧待处理
  TEST_F(FrameIngestTest, Ingest_StoresLidarPositionAndTime) {
    auto cloud = MakeCloud(0, 0, 0);
    voxel_map_.ingest(*cloud, {1.0, 2.0, 3.0}, 100.0);

    EXPECT_DOUBLE_EQ(voxel_map_.lidarPosition().x, 1.0);
    EXPECT_DOUBLE_EQ(voxel_map_.lidarPosition().y, 2.0);
    EXPECT_DOUBLE_EQ(voxel_map_.lidarPosition().z, 3.0);
    EXPECT_DOUBLE_EQ(voxel_map_.timestamp(), 100.0);
    EXPECT_TRUE(voxel_map_.hasPendingFrame());
  }

  // 首帧时刻即 system_init_time，故首帧的 elapsed 为 0
  TEST_F(FrameIngestTest, Ingest_FirstCall_SetsInitTime) {
    auto cloud = MakeCloud(0, 0, 0);
    voxel_map_.ingest(*cloud, {0.0, 0.0, 0.0}, 100.0);

    EXPECT_DOUBLE_EQ(initTime(), 100.0);
    EXPECT_DOUBLE_EQ(voxel_map_.elapsedSeconds(), 0.0);
  }

  // 观测时刻写进 intensity：体素叶靠它判年龄，首帧为 0、第二帧为两帧之差
  TEST_F(FrameIngestTest, Ingest_StampsObservationTimeIntoIntensity) {
    auto cloud = MakeCloud(0, 0, 0);
    voxel_map_.ingest(*cloud, {0.0, 0.0, 0.0}, 100.0);
    ASSERT_EQ(voxel_map_.frameCloud().points.size(), 1U);
    EXPECT_FLOAT_EQ(voxel_map_.frameCloud().points[0].intensity, 0.0F);

    voxel_map_.ingest(*MakeCloud(0, 0, 0), {0.0, 0.0, 0.0}, 100.5);
    ASSERT_EQ(voxel_map_.frameCloud().points.size(), 1U);
    EXPECT_FLOAT_EQ(voxel_map_.frameCloud().points[0].intensity, 0.5F);
    EXPECT_DOUBLE_EQ(voxel_map_.elapsedSeconds(), 0.5);
  }

  // 超出体素网格接收范围的点被裁剪掉
  TEST_F(FrameIngestTest, Ingest_FiltersPointsBeyondVoxelRange) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->push_back({0, 0, 0, 0});
    cloud->push_back({50, 50, 0, 0});  // 远超接收半径

    voxel_map_.ingest(*cloud, {0.0, 0.0, 0.0}, 100.0);

    EXPECT_EQ(voxel_map_.frameCloud().points.size(), 1U);
  }

  // update() 处理掉本帧后，待处理标记被清掉
  TEST_F(FrameIngestTest, Update_ClearsPendingFlag) {
    auto cloud = MakeCloud(0, 0, 0);
    voxel_map_.ingest(*cloud, {0.0, 0.0, 0.0}, 100.0);
    voxel_map_.update();

    EXPECT_FALSE(voxel_map_.hasPendingFrame());
  }
}  // namespace terrain_analysis
