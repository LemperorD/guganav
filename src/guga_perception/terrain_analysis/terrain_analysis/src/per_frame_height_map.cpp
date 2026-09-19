// 主线位置：一帧的后半程（节点在采集之后调用 compute）。
//
// 管线后半段：地面高程的收集与估计、障碍输出。
//
// 输入只有两样——采集点云与雷达位置——都由调用方逐帧传入，本类不保留帧间状态。

#include "terrain_analysis/per_frame_height_map.hpp"

#include "terrain_analysis/grid.hpp"
#include "terrain_analysis/grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace terrain_analysis {

  void PerFrameHeightMap::compute(const Cell& terrain_cloud,
                                  const guga_common::Point3d& lidar_position) {
    // 依赖是串联的：候选 → 逐格高程 → 输出。顺序不可换。
    estimateTerrainGround(terrain_cloud, lidar_position);
    computePlanarElevation();
    computeHeightMap(terrain_cloud, lidar_position);
  }

  void PerFrameHeightMap::estimateTerrainGround(
      const Cell& terrain_cloud, const guga_common::Point3d& lidar_position) {
    // 本阶段拥有候选集；逐格高程由 computePlanarElevation
    // 清，两份数据各有属主。
    for (auto& point_elevations : point_elev_) {
      point_elevations.clear();
    }

    for (const auto& point : terrain_cloud.points) {
      if (!aboveGroundFloor(point.z, config_)) {
        continue;
      }

      const GridIndex grid_index = gridIndex(
          point.x, point.y, lidar_position.x, lidar_position.y,
          config_.planar_voxel_size, PerFrameHeightGrid::WIDTH);
      if (!grid_index.valid) {
        continue;
      }

      addToNeighborhood3x3<PerFrameHeightGrid>(grid_index.row, grid_index.col,
                                               point.z, point_elev_);
    }
  }

  void PerFrameHeightMap::computePlanarElevation() {
    // 本阶段拥有逐格高程：没有候选的格保持 0（见 computeHeightMap 的说明）。
    voxel_elev_.fill(0);

    if (config_.use_sorting) {
      for (int i = 0; i < PerFrameHeightGrid::NUM; i++) {
        elevateByQuantile(i);
      }
    } else {
      for (int i = 0; i < PerFrameHeightGrid::NUM; i++) {
        elevateByMinimum(i);
      }
    }
  }

  void PerFrameHeightMap::computeHeightMap(
      const Cell& terrain_cloud, const guga_common::Point3d& lidar_position) {
    const double lidar_z = lidar_position.z;
    auto& elevations = obstacle_cloud_;
    elevations->clear();

    for (const auto& point : terrain_cloud.points) {
      const GridIndex grid_index = gridIndex(
          point.x, point.y, lidar_position.x, lidar_position.y,
          config_.planar_voxel_size, PerFrameHeightGrid::WIDTH);
      if (!grid_index.valid) {
        continue;
      }
      const size_t cell = PerFrameHeightGrid::linearIndex(grid_index.row,
                                                          grid_index.col);
      // 地面系：距本格局部地面的高度（voxel_elev_ 里是 odom 绝对值）。
      const double h_ground = point.z - voxel_elev_[cell];

      if (!abovePenetrationFloor(point.z - lidar_z, config_)) {
        continue;
      }
      if (!insideOutputBand(h_ground, config_)) {
        continue;
      }
      // 带内还要凑够该格的点数；这一条与参考系无关，是密度门限。
      const auto point_count = point_elev_[cell].size();
      if (point_count >= static_cast<size_t>(config_.min_block_point_num)) {
        const double height = config_.consider_drop ? std::abs(h_ground)
                                                    : h_ground;
        elevations->push_back(point);
        elevations->back().intensity = static_cast<float>(height);
      }
    }
  }

  bool PerFrameHeightMap::aboveGroundFloor(double z_odom,
                                           const PerFrameHeightConfig& config) {
    return z_odom > config.ground_floor_z;
  }

  bool PerFrameHeightMap::insideOutputBand(double h_ground,
                                           const PerFrameHeightConfig& config) {
    // 上界用带符号值：净空是"地面到障碍下沿"，低于地面的坑不占净空。
    if (h_ground >= config.ceiling_clearance) {
      return false;
    }
    // 死区下界用绝对值：considerDrop 打开时凹坑也算障碍。
    const double height = config.consider_drop ? std::abs(h_ground) : h_ground;
    return height >= config.min_obstacle_height;
  }

  bool PerFrameHeightMap::abovePenetrationFloor(
      double z_rel_lidar, const PerFrameHeightConfig& config) {
    return z_rel_lidar > config.min_relative_z;
  }

  void PerFrameHeightMap::elevateByQuantile(int cell) {
    auto& elevations = point_elev_[cell];
    int point_count = static_cast<int>(elevations.size());
    if (point_count == 0) {
      return;
    }
    sort(elevations.begin(), elevations.end());

    int quantile_index = static_cast<int>(config_.quantile_z * point_count);
    if (quantile_index >= point_count) {
      quantile_index = point_count - 1;
    }
    double minimum_z = elevations[0];
    double quantile_z = elevations[quantile_index];
    voxel_elev_[cell] = config_.limit_ground_lift ? std::min(
                            quantile_z, minimum_z + config_.max_ground_lift)
                                                  : quantile_z;
  }

  void PerFrameHeightMap::elevateByMinimum(int cell) {
    auto& elevations = point_elev_[cell];
    if (elevations.empty()) {
      return;
    }
    voxel_elev_[cell] = *std::min_element(elevations.begin(), elevations.end());
  }

}  // namespace terrain_analysis
