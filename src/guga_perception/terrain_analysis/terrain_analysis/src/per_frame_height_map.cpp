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
      // 唯一的候选筛选是下界，且用**绝对 z**（odom）：地面在 odom 中大体水平，
      // 地板过滤只需挡住远低于地面的穿透点，用绝对量比"相对雷达"更贴合语义，
      // 也不随雷达上下抖动而移动。
      if (point.z <= config_.ground_floor_z) {
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
      // 下面所有高度判据都以本帧估计的该处地面高度为基准。
      const double ground_z = voxel_elev_[cell];
      const double height_above_ground = point.z - ground_z;

      // 下界：地板过滤（挡掉地面以下/穿透点）。此处用**相对雷达**的高度，
      // 因为要挡的是"远低于雷达"的穿透点，与地形无关。
      if (point.z - lidar_z <= config_.min_relative_z) {
        continue;
      }
      // 上界：距地面达到安全间隙的点（天花板/横梁）不输出为障碍——
      // 净空足够时车辆可从下方通过（隧道场景）。以**局部地面**为基准：
      // 净空是"地面到障碍下沿"的距离，这也使判据在坡面上保持一致。
      // 这是障碍输出**唯一**的上界：不再叠加按车高的截断，否则车高与净空
      // 之间的那一带（车高 0.52 → 净空 0.62 之间）会被漏检，而车过不去。
      if (height_above_ground >= config_.ceiling_clearance) {
        continue;
      }
      double height = height_above_ground;
      if (config_.consider_drop) {
        height = std::abs(height);
      }

      auto point_count = point_elev_[cell].size();
      // 下界：地面带的死区，吸收地面高度估计的误差。估计值偏低时，真实地面点会
      // 算出几厘米的正高度；若从 0 起算，它们会被当作低矮障碍标记出去。
      if (height >= config_.min_obstacle_height
          && point_count >= static_cast<size_t>(config_.min_block_point_num)) {
        elevations->push_back(point);
        elevations->back().intensity = static_cast<float>(height);
      }
    }
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
