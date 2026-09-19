// 叶宽（scanVoxelSize）影响的测量工具，不属于 gtest、不参与 colcon test。
//
// 用法：measure_leaf [leaf_xy] [leaf_z] [frames] [ghost_frame]
// 场景：±5 m 地面（0.1 m 网格）+ 2 m 处 6 cm 矮台阶 + 3 m 处 0.1~0.5 m 的箱子。
// 输出：稳态后每帧耗时、输出点数、矮台阶与箱子各自的输出点数。
//
// 关注点：叶宽变大后，同一个 3D 叶里地面点与物体点会混合，而每个叶只保留
// 最新的一个点，矮台阶可能被地面点顶掉。
//
// 它直接构造两半并逐帧驱动，不经节点：这里量的是算法开销；ROS 接线与参数分发
// 由 test_integration 与 test_terrain_analysis 覆盖。

#include "terrain_analysis/core/per_frame_height_map.hpp"
#include "terrain_analysis/core/persistent_voxel_map.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

namespace {

  using terrain_analysis::PerFrameHeightMap;
  using terrain_analysis::PersistentVoxelMap;

  pcl::PointCloud<pcl::PointXYZI>::Ptr makeFrame(bool with_ghost_box = false) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    constexpr int kGrid = 101;  // ±5 m，0.1 m 间距
    for (int ix = 0; ix < kGrid; ix++) {
      for (int iy = 0; iy < kGrid; iy++) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(ix * 0.1 - 5.0);
        p.y = static_cast<float>(iy * 0.1 - 5.0);
        p.z = 0.0F;  // 地面
        p.intensity = 0.0F;
        cloud->push_back(p);
      }
    }
    // 2 m 处的 6 cm 矮台阶：0.1 m × 0.1 m × 6 cm，顶面 0.06 m
    for (int ix = 0; ix < 4; ix++) {
      for (int iy = 0; iy < 4; iy++) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(1.95 + ix * 0.02);
        p.y = static_cast<float>(-0.05 + iy * 0.02);
        p.z = 0.06F;
        p.intensity = 0.0F;
        cloud->push_back(p);
      }
    }
    // 幽灵点场景：4.1 m 处一个 0.3 m 高的方块，前若干帧存在，之后消失
    if (with_ghost_box) {
      for (int ix = 0; ix < 6; ix++) {
        for (int iz = 0; iz < 16; iz++) {
          pcl::PointXYZI p;
          p.x = static_cast<float>(4.1 + ix * 0.04);
          p.y = 0.0F;
          p.z = static_cast<float>(0.05 + iz * 0.02);  // 0.05~0.35 m
          p.intensity = 0.0F;
          cloud->push_back(p);
        }
      }
    }
    // 3 m 处的箱子：0.4 m 宽，高度从 0.1 到 0.5 m
    for (int ix = 0; ix < 9; ix++) {
      for (int iz = 0; iz < 21; iz++) {
        pcl::PointXYZI p;
        p.x = static_cast<float>(2.8 + ix * 0.05);
        p.y = 0.0F;
        p.z = static_cast<float>(0.1 + iz * 0.02);
        p.intensity = 0.0F;
        cloud->push_back(p);
      }
    }
    return cloud;
  }

  /** @brief 统计输出点云里落在给定平面邻域内的点数。 */
  size_t countNear(const pcl::PointCloud<pcl::PointXYZI>& cloud, double x,
                   double y, double radius) {
    size_t n = 0;
    for (const auto& p : cloud.points) {
      if (std::hypot(p.x - x, p.y - y) <= radius) {
        n++;
      }
    }
    return n;
  }

}  // namespace

int main(int argc, char** argv) {
  const double leaf_xy = argc > 1 ? std::atof(argv[1]) : 0.1;
  const double leaf_z = argc > 2 ? std::atof(argv[2]) : 0.05;
  const int frames = argc > 3 ? std::atoi(argv[3]) : 300;
  // 第 4 个参数：幽灵点场景中"物体消失"的帧号（<0 表示不启用该场景）
  const int ghost_frame = argc > 4 ? std::atoi(argv[4]) : -1;

  PersistentVoxelMap voxel_map;
  PerFrameHeightMap height_map;
  // 配置由调用方持有并按调用注入；取值与实车默认一致（见
  // config/reality/base.yaml）
  PersistentVoxelConfig voxel_config;
  PerFrameHeightConfig height_config;
  voxel_config.scan_voxel_size = leaf_xy;
  voxel_config.scan_voxel_size_z = leaf_z;
  voxel_config.decay_time = 0.5;
  voxel_config.no_decay_distance = 0.0;
  height_config.use_sorting = true;
  height_config.quantile_z = 0.2;
  height_config.min_obstacle_height = 0.04;
  height_config.ceiling_clearance = 0.62;

  const guga_common::Point3d lidar_position{0.0, 0.0, 0.0};
  const bool ghost_mode = ghost_frame >= 0;
  const auto frame = makeFrame(ghost_mode);  // 幽灵场景下含 4.1 m 方块
  const auto frame_no_ghost = makeFrame(false);  // 方块消失后的帧
  pcl::PointCloud<pcl::PointXYZI> collected;

  double total_ms = 0.0;
  int timed = 0;
  int ghost_clear_frame = -1;
  for (int i = 0; i < frames; i++) {
    const double t = i * 0.1;  // 10 Hz
    const bool ghost_present = ghost_frame < 0 || i < ghost_frame;
    // 与生产路径一致：前半段收帧并维护体素地图，采集结果交给后半段。
    voxel_map.ingest(ghost_present ? *frame : *frame_no_ghost, lidar_position,
                     t, voxel_config);

    const auto t0 = std::chrono::steady_clock::now();
    voxel_map.update(voxel_config);
    voxel_map.collectCloud(collected);
    height_map.compute(collected, voxel_map.lidarPosition(), height_config);
    const auto t1 = std::chrono::steady_clock::now();
    if (ghost_frame >= 0 && i >= ghost_frame && ghost_clear_frame < 0
        && countNear(height_map.obstacleCloud(), 4.2, 0.0, 0.3) == 0) {
      ghost_clear_frame = i;  // 幽灵点从输出中消失的帧号
    }
    if (i >= 100) {  // 跳过预热
      total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
      timed++;
    }
  }

  const auto& out = height_map.obstacleCloud();
  std::printf(
      "leaf_xy=%.3f leaf_z=%.3f  frames=%d\n"
      "  单帧 update+输出: %.3f ms（稳态 %d 帧均值，不含收帧裁剪）\n"
      "  输出点数        : %zu\n"
      "  矮台阶(2m,6cm)  : %zu 点\n"
      "  箱子(3m,0.1-0.5): %zu 点\n",
      leaf_xy, leaf_z, frames, total_ms / (timed > 0 ? timed : 1), timed,
      out.points.size(), countNear(out, 2.0, 0.0, 0.15),
      countNear(out, 3.0, 0.0, 0.3));

  if (ghost_frame >= 0) {
    const int latency = ghost_clear_frame < 0 ? -1
                                              : ghost_clear_frame - ghost_frame;
    if (latency < 0) {
      std::printf("  幽灵点(4.2m)    : 移除于第 %d 帧，此后一直未消失\n",
                  ghost_frame);
    } else {
      std::printf(
          "  幽灵点(4.2m)    : 移除于第 %d 帧，输出清空延迟 %d 帧（%.1f s "
          "@10Hz）\n",
          ghost_frame, latency, latency * 0.1);
    }
  }

  return 0;
}
