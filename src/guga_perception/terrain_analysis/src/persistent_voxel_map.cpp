// 主线位置：一帧的前半程（节点按 receiveFrame → update → collectCloud 调用）。

#include "terrain_analysis/persistent_voxel_map.hpp"

#include "terrain_analysis/grid.hpp"
#include "terrain_analysis/grid_utils.hpp"

#include <unordered_map>

namespace terrain_analysis {

  std::array<PersistentVoxelMap::Cell::Ptr, PersistentVoxelGrid::NUM>
  PersistentVoxelMap::makeCells() {
    std::array<Cell::Ptr, PersistentVoxelGrid::NUM> cells;
    for (auto& ptr : cells) {
      ptr = std::make_shared<Cell>();
    }
    return cells;
  }

  void PersistentVoxelMap::receiveFrame(
      const Cell& cloud, const guga_common::Point3d& lidar_position,
      double timestamp_sec) {
    lidar_ = lidar_position;
    time_ = timestamp_sec;
    if (!inited_) {
      init_time_ = time_;
      inited_ = true;
    }

    const double elapsed = elapsedSeconds();
    const double max_range = config_.terrain_voxel_size
                             * (PersistentVoxelGrid::HALF_WIDTH + 1);
    frame_cloud_->clear();
    for (const auto& point : cloud.points) {
      // 高度取雷达系。
      const double z_rel_lidar = point.z - lidar_.z;
      // 距离与参考系无关。
      const double distance = horizontalDistance(point.x, point.y, lidar_.x,
                                                 lidar_.y);
      if (insideReceiveBand(z_rel_lidar, distance) && distance < max_range) {
        pcl::PointXYZI cropped = point;
        // intensity 借来携带观测时刻，rebuild 据此判年龄。
        cropped.intensity = static_cast<float>(elapsed);
        frame_cloud_->push_back(cropped);
      }
    }

    frame_pending_ = true;
  }

  void PersistentVoxelMap::update() {
    frame_pending_ = false;
    rollover();
    addFrame();
    rebuildGrids();
  }

  void PersistentVoxelMap::rollover() {
    const guga_common::Point3d& lidar = lidar_;
    const double voxel_size = config_.terrain_voxel_size;
    double center_x = voxel_size * shift_x_;
    double center_y = voxel_size * shift_y_;

    while (lidar.x - center_x < -voxel_size) {
      shift(ShiftAxis::X, ShiftDirection::NEGATIVE);
      center_x = voxel_size * --shift_x_;
    }
    while (lidar.x - center_x > voxel_size) {
      shift(ShiftAxis::X, ShiftDirection::POSITIVE);
      center_x = voxel_size * ++shift_x_;
    }
    while (lidar.y - center_y < -voxel_size) {
      shift(ShiftAxis::Y, ShiftDirection::NEGATIVE);
      center_y = voxel_size * --shift_y_;
    }
    while (lidar.y - center_y > voxel_size) {
      shift(ShiftAxis::Y, ShiftDirection::POSITIVE);
      center_y = voxel_size * ++shift_y_;
    }
  }

  void PersistentVoxelMap::shift(ShiftAxis axis, ShiftDirection positive) {
    static constexpr int WIDTH = PersistentVoxelGrid::WIDTH;
    const bool toward_positive = positive == ShiftDirection::POSITIVE;
    const int src = toward_positive ? 0 : WIDTH - 1;
    const int dst = toward_positive ? WIDTH - 1 : 0;
    const int step = toward_positive ? 1 : -1;

    // 沿 x 搬运变化的是列下标，沿 y 搬运变化的是行下标。
    const bool along_x = axis == ShiftAxis::X;
    for (int fixed = 0; fixed < WIDTH; fixed++) {
      const auto cell = [&](int m) {
        return along_x ? PersistentVoxelGrid::linearIndex(fixed, m)
                       : PersistentVoxelGrid::linearIndex(m, fixed);
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

  void PersistentVoxelMap::addFrame() {
    const Cell& cell = *frame_cloud_;
    const guga_common::Point3d& lidar = lidar_;

    for (const auto& point : cell.points) {
      const GridIndex index = gridIndex(point.x, point.y, lidar.x, lidar.y,
                                        config_.terrain_voxel_size,
                                        PersistentVoxelGrid::WIDTH);
      if (!index.valid) {
        continue;
      }
      cloud_[PersistentVoxelGrid::linearIndex(index.row, index.col)]->push_back(
          point);
    }
  }

  void PersistentVoxelMap::rebuildGrids() {
    const guga_common::Point3d& lidar = lidar_;
    const double now_elapsed = elapsedSeconds();
    std::unordered_map<uint64_t, size_t> leaf_slot;
    Cell representatives;

    for (auto& cell_ptr : cloud_) {
      Cell& cell = *cell_ptr;

      representatives.clear();
      leaf_slot.clear();

      // 逐叶保留最新观测
      for (const auto& point : cell.points) {
        const uint64_t key = leafKey(point.x, point.y, point.z);
        const auto it = leaf_slot.find(key);
        if (it == leaf_slot.end()) {
          leaf_slot.emplace(key, representatives.size());
          representatives.push_back(point);
        } else if (point.intensity > representatives[it->second].intensity) {
          representatives[it->second] = point;
        }
      }

      cell.clear();
      // 按接收带与年龄过滤。
      for (const auto& point : representatives.points) {
        const double distance = horizontalDistance(point.x, point.y, lidar.x,
                                                   lidar.y);
        if (keepPoint(point.z - lidar.z, distance, point.intensity,
                      now_elapsed)) {
          cell.push_back(point);
        }
      }
    }
  }

  uint64_t PersistentVoxelMap::leafKey(double x, double y, double z) {
    auto leaf_xy = config_.scan_voxel_size;
    auto leaf_z = config_.scan_voxel_size_z;
    // 哈希函数: 位置/叶长 取整 + 偏置 kBias.
    // 该对应关系确保该哈希键为正值,防止接下来的左移出现未定义错误。
    constexpr int kBits = 21;
    constexpr int64_t kBias = 1000000;
    const auto index = [](double value, double leaf) {
      return static_cast<uint64_t>(
          static_cast<int64_t>(std::floor(value / leaf)) + kBias);
    };
    // 哈希键: 返回 [x段哈希][y段哈希][z段哈希]
    // 拼接的位运算, 确保坐标的哈希值唯一.
    return (index(x, leaf_xy) << (2 * kBits)) | (index(y, leaf_xy) << kBits)
           | index(z, leaf_z);
  }

  bool PersistentVoxelMap::keepPoint(double z_rel_lidar, double distance,
                                     double point_time, double now_elapsed) {
    if (!insideReceiveBand(z_rel_lidar, distance)) {
      return false;
    }
    return (now_elapsed - point_time) < config_.decay_time;
  }

  bool PersistentVoxelMap::insideReceiveBand(double z_rel_lidar,
                                             double distance) {
    const double z_margin = config_.distance_ratio_z * distance;
    return z_rel_lidar > config_.min_relative_z - z_margin
           && z_rel_lidar < config_.max_relative_z + z_margin;
  }

  void PersistentVoxelMap::collectCloud(Cell& out) const {
    // 这里只决定窗口多大，格到点云的换算见 grid_utils.hpp。
    collectWindow<PersistentVoxelGrid>(cloud_, EXTRACT_HALF_WINDOW, out);
  }

}  // namespace terrain_analysis
