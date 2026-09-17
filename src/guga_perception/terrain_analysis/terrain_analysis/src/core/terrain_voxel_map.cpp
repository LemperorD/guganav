// 体素地图（跨帧持久）的维护：滚动、归格、按叶保留最新观测。
//
// 从 TerrainProcessor 提取为类，使"网格 + 窗口偏移"成为一份可单独构造、单独
// 驱动的状态：所有输入（雷达位姿、当前时刻、配置）由调用方显式传入，不读任何
// 全局状态。

#include "terrain_analysis/core/terrain_voxel_map.hpp"

#include "terrain_analysis/core/grid_lookup.hpp"

#include <unordered_map>

namespace terrain_analysis {

  std::array<TerrainVoxelMap::Cell::Ptr, TerrainGrid::TERRAIN_VOXEL_NUM>
  TerrainVoxelMap::makeCells() {
    std::array<Cell::Ptr, TerrainGrid::TERRAIN_VOXEL_NUM> cells;
    for (auto& ptr : cells) {
      ptr = std::make_shared<Cell>();
    }
    return cells;
  }

  uint64_t TerrainVoxelMap::leafKey(double x, double y, double z,
                                    double leaf_xy, double leaf_z) {
    // O(n) 融合所需的叶键：坐标除以叶宽取整后按 21 bit 打包。
    // 偏置 10^6 使 odom 负坐标也能装下，覆盖约 ±54 km 的运行范围。
    constexpr int kBits = 21;
    constexpr int64_t kBias = 1000000;
    const auto index = [](double value, double leaf) {
      return static_cast<uint64_t>(
          static_cast<int64_t>(std::floor(value / leaf)) + kBias);
    };
    return (index(x, leaf_xy) << (2 * kBits)) | (index(y, leaf_xy) << kBits)
           | index(z, leaf_z);
  }

  bool TerrainVoxelMap::keepPoint(double relative_z, double distance,
                                  double point_time,
                                  const TerrainConfig& config,
                                  double now_elapsed) {
    const double z_margin = config.distance_ratio_z * distance;
    if (relative_z <= config.min_relative_z - z_margin) {
      return false;
    }
    if (relative_z >= config.max_relative_z + z_margin) {
      return false;
    }
    const bool near = distance < config.no_decay_distance;
    const bool decayed = (now_elapsed - point_time) >= config.decay_time;
    return !(decayed && !near);
  }

  void TerrainVoxelMap::update(const Cell& crop, const LidarPose& lidar,
                               double now_elapsed,
                               const TerrainConfig& config) {
    rollover(lidar, config.terrain_voxel_size);
    addFrame(crop, lidar, config.terrain_voxel_size);
    rebuild(config, lidar, now_elapsed);
  }

  void TerrainVoxelMap::collectCloud(Cell& out) const {
    out.clear();
    constexpr int HALF = TerrainGrid::TERRAIN_VOXEL_HALF_WIDTH;
    for (int row = HALF - EXTRACT_HALF_WINDOW;
         row <= HALF + EXTRACT_HALF_WINDOW; row++) {
      for (int column = HALF - EXTRACT_HALF_WINDOW;
           column <= HALF + EXTRACT_HALF_WINDOW; column++) {
        out += *cloud_[TerrainGrid::terrainVoxelIndex(row, column)];
      }
    }
  }

  void TerrainVoxelMap::rollover(const LidarPose& lidar, double voxel_size) {
    double center_x = voxel_size * shift_x_;
    double center_y = voxel_size * shift_y_;

    while (lidar.x - center_x < -voxel_size) {
      shift(true, false);
      center_x = voxel_size * --shift_x_;
    }
    while (lidar.x - center_x > voxel_size) {
      shift(true, true);
      center_x = voxel_size * ++shift_x_;
    }
    while (lidar.y - center_y < -voxel_size) {
      shift(false, false);
      center_y = voxel_size * --shift_y_;
    }
    while (lidar.y - center_y > voxel_size) {
      shift(false, true);
      center_y = voxel_size * ++shift_y_;
    }
  }

  void TerrainVoxelMap::addFrame(const Cell& crop, const LidarPose& lidar,
                                 double voxel_size) {
    for (const auto& point : crop.points) {
      const GridIndex index = gridIndex(point.x, point.y, lidar.x, lidar.y,
                                        voxel_size,
                                        TerrainGrid::TERRAIN_VOXEL_WIDTH);
      if (!index.valid) {
        continue;
      }
      cloud_[TerrainGrid::terrainVoxelIndex(index.row, index.col)]->push_back(
          point);
    }
  }

  void TerrainVoxelMap::rebuild(const TerrainConfig& config,
                                const LidarPose& lidar, double now_elapsed) {
    // 每个格子每帧重建一次，逐叶只保留"观测时刻最新"的那一个点。
    //
    // 时刻取最新而不是平均：叶内混有新老点时，平均会把仍在被观测的表面判成
    // 过期（这曾由 PCL VoxelGrid 的质心 + intensity 平均引入）。
    //
    // "有新点即刷新、无新点才判年龄"因此不需要额外状态：代表点自带的时刻就是
    // 该叶的 last_seen；有本帧新点进来时代表点会被换成新点，没有新点时保留
    // 上一轮时刻，由 keepPoint 判年龄。
    std::unordered_map<uint64_t, size_t> leaf_slot;
    Cell representatives;

    for (auto& cell_ptr : cloud_) {
      Cell& cell = *cell_ptr;

      representatives.clear();
      leaf_slot.clear();

      for (const auto& point : cell.points) {
        const uint64_t key = leafKey(point.x, point.y, point.z,
                                     config.scan_voxel_size,
                                     config.scan_voxel_size_z);
        const auto it = leaf_slot.find(key);
        if (it == leaf_slot.end()) {
          leaf_slot.emplace(key, representatives.size());
          representatives.push_back(point);
        } else if (point.intensity > representatives[it->second].intensity) {
          representatives[it->second] = point;
        }
      }

      cell.clear();
      for (const auto& point : representatives.points) {
        const double distance = horizontalDistance(point.x, point.y, lidar.x,
                                                   lidar.y);
        if (keepPoint(point.z - lidar.z, distance, point.intensity, config,
                      now_elapsed)) {
          cell.push_back(point);
        }
      }
    }
  }

  void TerrainVoxelMap::shift(bool along_x, bool toward_positive) {
    static constexpr int WIDTH = TerrainGrid::TERRAIN_VOXEL_WIDTH;
    const int src = toward_positive ? 0 : WIDTH - 1;
    const int dst = toward_positive ? WIDTH - 1 : 0;
    const int step = toward_positive ? 1 : -1;

    for (int fixed = 0; fixed < WIDTH; fixed++) {
      const auto cell = [&](int m) {
        return along_x ? TerrainGrid::terrainVoxelIndex(m, fixed)
                       : TerrainGrid::terrainVoxelIndex(fixed, m);
      };
      auto ptr = cloud_[cell(src)];
      for (int m = src; m != dst; m += step) {
        cloud_[cell(m)] = cloud_[cell(m + step)];
      }
      auto& dst_cell = cloud_[cell(dst)];
      dst_cell = ptr;
      dst_cell->clear();
    }
  }

}  // namespace terrain_analysis
