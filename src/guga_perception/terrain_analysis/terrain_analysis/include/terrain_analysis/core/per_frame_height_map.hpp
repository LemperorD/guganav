#pragma once

#include "terrain_analysis/core/per_frame_height_config.hpp"
#include "terrain_analysis/core/per_frame_height_grid.hpp"
#include "guga_common/geometry.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <memory>
#include <vector>

namespace terrain_analysis {

  /**
   * @brief 管线后半段：逐帧的地面高程估计与障碍输出。
   *
   * 两半的分界是**寿命**：本类逐帧重建，不保留任何帧间状态；前半段
   * PersistentVoxelMap 跨帧持续，是唯一保留帧间状态的一侧。
   *
   * 持有一张 51×51 的网格（每格的地面候选高度与被估计出的地面高度）与输出点云，
   * 每帧从头填一遍；逐帧输入（采集点云、雷达位置）由调用方显式传入。
   *
   * 三段串联，顺序不可换（后者都依赖前者的产物）：
   *   estimateTerrainGround → computePlanarElevation → computeHeightMap
   *
   * 可见性契约：三段是实现细节，一律 private，可自由重构签名而不影响调用方；
   * 外部只走 compute() 与 obstacleCloud()。需要白盒验证各段的测试通过 friend
   * 显式获得访问权，刻意不把这些阶段提升为公开 API——否则内部顺序会固化成对外
   * 契约。
   */
  class PerFrameHeightMap {
  public:
    /** @brief 点云容器类型（与前半段的格子同一类型）。 */
    using Cell = pcl::PointCloud<pcl::PointXYZI>;

    PerFrameHeightMap() = default;

    PerFrameHeightMap(const PerFrameHeightMap&) = delete;
    PerFrameHeightMap& operator=(const PerFrameHeightMap&) = delete;
    PerFrameHeightMap(PerFrameHeightMap&&) = delete;
    PerFrameHeightMap& operator=(PerFrameHeightMap&&) = delete;

    /** @brief 可修改的后半段参数。 */
    [[nodiscard]] PerFrameHeightConfig& config() noexcept {
      return config_;
    }
    /** @brief 只读的后半段参数。 */
    [[nodiscard]] const PerFrameHeightConfig& config() const noexcept {
      return config_;
    }

    /**
     * @brief 跑完整后半段：估地面 → 逐格高程 → 障碍输出。
     * @param terrain_cloud 采集点云（前半段 collectCloud 的产物）。
     * @param lidar_position 雷达在 odom
     * 下的位置（不是车体位置），作为平面网格锚点。
     */
    void compute(const Cell& terrain_cloud,
                 const guga_common::Point3d& lidar_position);

    /** @brief 最近一次生成的障碍点云（intensity 为距局部地面的高度）。 */
    [[nodiscard]] const Cell& obstacleCloud() const noexcept {
      return *obstacle_cloud_;
    }

  private:
    /**
     * @brief 收集地面高度候选：把每个地面点膨胀到 3×3 平面邻域。
     * @param terrain_cloud 采集点云。
     * @param lidar_position 雷达位置，作为平面网格锚点。
     */
    void estimateTerrainGround(const Cell& terrain_cloud,
                               const guga_common::Point3d& lidar_position);

    /** @brief 逐格估计地面高度（分位数或最小值，见配置）。 */
    void computePlanarElevation();

    /**
     * @brief 生成障碍输出：离地高度落在输出带内的点写入 intensity。
     * @param terrain_cloud 采集点云。
     * @param lidar_position 雷达位置，用于挡掉远低于雷达的穿透点。
     */
    void computeHeightMap(const Cell& terrain_cloud,
                          const guga_common::Point3d& lidar_position);

    /**
     * @brief 把一个高度值加入指定平面格及其 3×3 邻域的候选集中。
     *
     * 邻域膨胀的目的是让每格地面高度由约 0.6 m 范围内的点共同决定，以抗单点
     * 噪声；代价有二，改动时需一并考虑：
     *   - 实际地面分辨率低于标称的 planar_voxel_size（0.2 m）；
     *   - min_block_point_num 的语义被稀释——只靠邻居的点也能凑够该阈值。
     * @param row 中心格的行下标（必须已在网格范围内）。
     * @param col 中心格的列下标（必须已在网格范围内）。
     * @param z 要加入候选的高度值（odom 绝对 z）。
     */
    void addToPlanarNeighborhood3x3(int row, int col, double z);
    /** @brief 用分位数估计指定平面格的地面高度。 */
    void elevateByQuantile(int cell);
    /** @brief 用最低点估计指定平面格的地面高度。 */
    void elevateByMinimum(int cell);

    PerFrameHeightConfig config_;

    /** @brief 每格收集到的地面高度候选值。 */
    std::array<std::vector<double>, PerFrameHeightGrid::NUM> point_elev_;
    /** @brief 每格估计出的地面高度（没有候选的格保持 0）。 */
    std::array<double, PerFrameHeightGrid::NUM> voxel_elev_{};
    /** @brief 障碍输出点云。 */
    Cell::Ptr obstacle_cloud_ = std::make_shared<Cell>();

    // 白盒测试需要逐阶段驱动与检查内部状态；仅授予本包测试 fixture，
    // 不对外开放（新增测试如需访问，在此显式追加）。
    friend class AlgorithmTest;
  };

}  // namespace terrain_analysis
