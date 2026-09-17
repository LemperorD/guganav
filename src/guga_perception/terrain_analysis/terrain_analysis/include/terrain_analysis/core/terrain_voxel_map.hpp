#pragma once

#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/grid.hpp"
#include "terrain_analysis/core/lidar_pose.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <memory>

namespace terrain_analysis {

  /**
   * @brief 跨帧持久的体素地图：累积点云 + 以雷达为中心的滚动窗口。
   *
   * 这是管线里唯一跨帧保留的点云数据。逐帧数据（裁剪点云、采集点云、平面高程
   * 等）不属于这里。
   *
   * 内部只有一种锚点来源：生产路径走 update()，它在一帧内用同一个雷达位姿完成
   * "滚动 → 归格 → 重建"三步，调用方无法把不同的锚点混进同一帧。逐阶段方法
   * （rollover / addFrame / rebuild）单独开放，供白盒测试与后续阶段使用。
   */
  class TerrainVoxelMap {
  public:
    /** @brief 一格累积点云。 */
    using Cell = pcl::PointCloud<pcl::PointXYZI>;

    TerrainVoxelMap() = default;

    /**
     * @brief 一帧的完整维护：滚动窗口 → 本帧点云归格 → 逐格重建。
     * @param crop 本帧裁剪点云（intensity 已由 ingest 写入观测时刻）。
     * @param lidar 雷达位姿。
     * @param now_elapsed 当前时刻（相对首帧，秒）。
     */
    void update(const Cell& crop, const LidarPose& lidar, double now_elapsed,
                const TerrainConfig& config);

    /** @brief 雷达移动时滚动网格，维持以雷达为中心的窗口。 */
    void rollover(const LidarPose& lidar, double voxel_size);

    /** @brief 把本帧点云按位置分配到体素格。 */
    void addFrame(const Cell& crop, const LidarPose& lidar, double voxel_size);

    /**
     * @brief 逐格重建：按叶保留最新观测点，并做高度带与年龄过滤。
     *
     * 每个叶（水平 scan_voxel_size、垂直 scan_voxel_size_z）只保留观测时刻最新
     * 的那一个点，于是"有新点即刷新、无新点才判年龄"不需要额外状态：代表点自带
     * 的时刻就是该叶的 last_seen。
     */
    void rebuild(const TerrainConfig& config, const LidarPose& lidar,
                 double now_elapsed);

    /** @brief 只读访问所有格子。 */
    [[nodiscard]] const std::array<Cell::Ptr, TerrainGrid::TERRAIN_VOXEL_NUM>&
    cells() const noexcept {
      return cloud_;
    }
    /** @brief 可修改访问所有格子（供采集阶段与测试使用）。 */
    [[nodiscard]] std::array<Cell::Ptr, TerrainGrid::TERRAIN_VOXEL_NUM>&
    cells() noexcept {
      return cloud_;
    }

    /** @brief 网格相对初始中心的 x 方向偏移（格数）。 */
    [[nodiscard]] int shiftX() const noexcept {
      return shift_x_;
    }
    /** @brief 网格相对初始中心的 y 方向偏移（格数）。 */
    [[nodiscard]] int shiftY() const noexcept {
      return shift_y_;
    }

    /** @brief 融合叶键：x/y 与 z 使用不同叶宽（垂直更细）。 */
    [[nodiscard]] static uint64_t leafKey(double x, double y, double z,
                                          double leaf_xy, double leaf_z);

    /** @brief 该点是否应保留在该体素格中（高度带 + 年龄）。 */
    [[nodiscard]] static bool keepPoint(double relative_z, double distance,
                                        double point_time,
                                        const TerrainConfig& config,
                                        double now_elapsed);

  private:
    /** @brief 把整张网格沿指定轴搬运一格，腾出的新格清空。 */
    void shift(bool along_x, bool toward_positive);

    std::array<Cell::Ptr, TerrainGrid::TERRAIN_VOXEL_NUM> cloud_ = makeCells();
    int shift_x_ = 0;
    int shift_y_ = 0;

    [[nodiscard]] static std::array<Cell::Ptr, TerrainGrid::TERRAIN_VOXEL_NUM>
    makeCells();
  };

}  // namespace terrain_analysis
