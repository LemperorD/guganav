#pragma once

#include "terrain_analysis/core/terrain_voxel_config.hpp"
#include "terrain_analysis/core/terrain_voxel_grid.hpp"
#include "guga_common/geometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <memory>

namespace terrain_analysis {

  /**
   * @brief 管线前半段：本帧输入的接收与裁剪 + 跨帧持久的体素地图。
   *
   * 持有两样东西：
   *   - 跨帧保留的点云（管线里唯一一份），随雷达移动滚动；
   *   - 本帧输入：裁剪后的点云、雷达位置、帧时刻与首帧时刻。
   *
   * 裁剪放在这里而不是调用方，原因是那两条判据（高度带、接收半径）本来就在服务
   * 这张网格的接收范围：接收半径就是网格宽度，高度带与 rebuild 的 keepPoint
   * 同源。
   *
   * 一帧的流程：ingest() 收帧 → update() 滚动/归格/重建 → collectCloud()
   * 取窗口。 update() 用 ingest()
   * 记下的同一个锚点完成三步，调用方无法把不同的锚点混进
   * 同一帧；逐阶段方法（rollover / addFrame / rebuild）单独开放，供白盒测试与
   * 后续阶段用，锚点与时刻由参数传入。
   *
   * 线程模型：本类不做同步，调用方必须保证 ingest 与 update 不并发执行（当前由
   * 节点在单线程执行器中串行调用满足）。
   */
  class TerrainVoxelMap {
  public:
    /** @brief 一格累积点云。 */
    using Cell = pcl::PointCloud<pcl::PointXYZI>;

    TerrainVoxelMap() = default;

    /** @brief 可修改的前半段参数。 */
    [[nodiscard]] TerrainVoxelConfig& config() noexcept {
      return config_;
    }
    /** @brief 只读的前半段参数。 */
    [[nodiscard]] const TerrainVoxelConfig& config() const noexcept {
      return config_;
    }

    /**
     * @brief 接一帧点云：按高度带与接收半径裁剪，写入观测时刻。
     *
     * 裁剪同时记录本帧的雷达位置与时刻（首帧时刻也在此时记录），后续 update()
     * 与 elapsedSeconds() 都以它为准。
     * @param cloud 输入点云，坐标位于 odom 坐标系。
     * @param lidar_position 雷达在 odom 下的位置（不是车体位置）。
     * @param timestamp_sec 本帧时间戳，单位为秒。
     */
    void ingest(const Cell& cloud, const guga_common::Point3d& lidar_position,
                double timestamp_sec);

    /** @brief 是否存在尚未处理的一帧。 */
    [[nodiscard]] bool hasPendingFrame() const noexcept {
      return frame_pending_;
    }

    /**
     * @brief 本帧维护：滚动窗口 → 本帧点云归格 → 逐格重建。
     *
     * 使用 ingest() 记下的雷达位置、时刻与自身的配置，因此没有参数可传错。
     */
    void update();

    /**
     * @brief 采集地图中央窗口内的累积点云。
     *
     * 取以雷达为中心的 11x11 格（约 ±5.5 m）——窗口大小是网格布局自身的事实，
     * 因此由地图类持有，而不是让调用方记住。
     * @param out 输出容器，会被清空后填入采集结果。
     */
    void collectCloud(Cell& out) const;

    /** @brief 裁剪后的本帧点云（intensity 为观测时刻，相对首帧的秒数）。 */
    [[nodiscard]] const Cell& frameCloud() const noexcept {
      return *frame_cloud_;
    }

    /** @brief 最近一帧的雷达位置（odom 下）。 */
    [[nodiscard]] const guga_common::Point3d& lidarPosition() const noexcept {
      return lidar_;
    }

    /** @brief 最近一帧的时间戳，单位为秒。 */
    [[nodiscard]] double timestamp() const noexcept {
      return time_;
    }

    /** @brief 最近一帧相对首帧的秒数。 */
    [[nodiscard]] double elapsedSeconds() const noexcept {
      return time_ - init_time_;
    }

    /** @brief 雷达移动时滚动网格，维持以雷达为中心的窗口。 */
    void rollover(const guga_common::Point3d& lidar);

    /** @brief 把本帧点云按位置分配到体素格。 */
    void addFrame(const Cell& crop, const guga_common::Point3d& lidar);

    /**
     * @brief 逐格重建：按叶保留最新观测点，并做高度带与年龄过滤。
     *
     * 每个叶（水平 scan_voxel_size、垂直 scan_voxel_size_z）只保留观测时刻最新
     * 的那一个点，于是"有新点即刷新、无新点才判年龄"不需要额外状态：代表点自带
     * 的时刻就是该叶的 last_seen。
     */
    void rebuild(const guga_common::Point3d& lidar, double now_elapsed);

    /** @brief 只读访问所有格子。 */
    [[nodiscard]] const std::array<Cell::Ptr, TerrainVoxelGrid::NUM>& cells()
        const noexcept {
      return cloud_;
    }
    /** @brief 可修改访问所有格子（供采集阶段与测试使用）。 */
    [[nodiscard]] std::array<Cell::Ptr, TerrainVoxelGrid::NUM>&
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
                                        const TerrainVoxelConfig& config,
                                        double now_elapsed);

  private:
    /** @brief 采集窗口的半宽（格数）：以雷达为中心的 11x11 格。 */
    static constexpr int EXTRACT_HALF_WINDOW = 5;

    /** @brief 把整张网格沿指定轴搬运一格，腾出的新格清空。 */
    void shift(bool along_x, bool toward_positive);

    TerrainVoxelConfig config_;

    std::array<Cell::Ptr, TerrainVoxelGrid::NUM> cloud_ = makeCells();
    int shift_x_ = 0;
    int shift_y_ = 0;

    // 本帧输入：由 ingest 写入，update 消费。
    Cell::Ptr frame_cloud_ = std::make_shared<Cell>();
    guga_common::Point3d lidar_;
    double time_ = 0.0;
    double init_time_ = 0.0;
    bool inited_ = false;
    bool frame_pending_ = false;

    [[nodiscard]] static std::array<Cell::Ptr, TerrainVoxelGrid::NUM>
    makeCells();

    // 白盒测试需要逐阶段驱动并检查内部状态；仅授予本包测试 fixture，
    // 不对外开放（新增测试如需访问，在此显式追加）。
    friend class AlgorithmTest;
    friend class FrameIngestTest;
  };

}  // namespace terrain_analysis
