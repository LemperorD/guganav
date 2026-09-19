// 主线位置：一帧的前半程（节点按 ingest → update → collectCloud 调用）。
//
// 管线前半段：本帧输入的接收与裁剪 + 体素地图（跨帧持久）的维护。
//
// 本类不读任何全局状态：帧输入由 ingest() 写入，配置由调用方在启动时填入。

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

  void PersistentVoxelMap::ingest(const Cell& cloud,
                                  const guga_common::Point3d& lidar_position,
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
      // 高度取相对雷达（雷达系），距离与参考系无关。
      const double z_rel_lidar = point.z - lidar_.z;
      const double distance = horizontalDistance(point.x, point.y, lidar_.x,
                                                 lidar_.y);
      if (insideReceiveBand(z_rel_lidar, distance) && distance < max_range) {
        pcl::PointXYZI cropped = point;
        // intensity 借用来携带该点的观测时刻（相对首帧的秒数），rebuild 判年龄
        // 时读它；原始反射强度在下游没有被使用。
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
    const Cell& crop = *frame_cloud_;
    const guga_common::Point3d& lidar = lidar_;
    const double voxel_size = config_.terrain_voxel_size;

    for (const auto& point : crop.points) {
      const GridIndex index = gridIndex(point.x, point.y, lidar.x, lidar.y,
                                        voxel_size, PersistentVoxelGrid::WIDTH);
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
    // 拼接本身是"格 → 点云"的通用换算，见 grid_utils.hpp；这里只决定窗口多大。
    collectWindow<PersistentVoxelGrid>(cloud_, EXTRACT_HALF_WINDOW, out);
  }

}  // namespace terrain_analysis
