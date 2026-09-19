#pragma once

#include "terrain_analysis/config.hpp"
#include "terrain_analysis/grid.hpp"
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
   * 每帧从头填一遍；逐帧输入（采集点云、雷达位置）由调用方显式传入，配置只在
   * 构造时以常量引用注入——本类不修改它，也没有"可写配置"这种入口。
   *
   * 三段串联，顺序不可换（后者都依赖前者的产物）：
   *   estimateTerrainGround → computePlanarElevation → computeHeightMap
   *
   * 可见性契约：三段是实现细节，一律 private，可自由重构签名而不影响调用方；
   * 外部只走 compute() 与 obstacleCloud()。三段与网格数据放在 protected 的
   * "接缝"区，需要白盒验证的测试用派生类把它们提升为公有（见
   * test/test_doubles.hpp），生产头文件里不出现测试类名。
   */
  class PerFrameHeightMap {
  public:
    /** @brief 点云容器类型（与前半段的格子同一类型）。 */
    using Cell = pcl::PointCloud<pcl::PointXYZI>;

    /**
     * @brief 以配置构造：只保存常量引用，既不拷贝也不修改它。
     *
     * 引用而非副本，是为了让所有者（节点、测试 fixture、测量工具）改动自己那份
     * 配置后，本类下一次调用就能看到，不必重建对象。因此：
     *   - config 必须比本对象活得更久（所有者先声明、后销毁）；
     *   - 右值构造被显式删除，避免绑定到临时量；
     *   - 所有者改动配置的时机必须是"本对象不在运行中"——本类不做任何同步。
     */
    explicit PerFrameHeightMap(const PerFrameHeightConfig& config) noexcept
        : config_(config) {
    }
    /** @brief 禁止绑定临时配置：引用会立即悬垂。 */
    PerFrameHeightMap(PerFrameHeightConfig&&) = delete;

    PerFrameHeightMap(const PerFrameHeightMap&) = delete;
    PerFrameHeightMap& operator=(const PerFrameHeightMap&) = delete;
    PerFrameHeightMap(PerFrameHeightMap&&) = delete;
    PerFrameHeightMap& operator=(PerFrameHeightMap&&) = delete;

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

  protected:
    // ── 接缝：三段阶段与网格数据。生产代码不用，白盒测试用派生类提升为公有
    // （见 test/test_doubles.hpp），生产头文件里不出现测试类名。 ──
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

    /** @brief 每格收集到的地面高度候选值。 */
    std::array<std::vector<double>, PerFrameHeightGrid::NUM> point_elev_;
    /** @brief 每格估计出的地面高度（没有候选的格保持 0）。 */
    std::array<double, PerFrameHeightGrid::NUM> voxel_elev_{};
    /** @brief 障碍输出点云。 */
    Cell::Ptr obstacle_cloud_ = std::make_shared<Cell>();

  private:
    /** @brief 用分位数估计指定平面格的地面高度。 */
    void elevateByQuantile(int cell);
    /** @brief 用最低点估计指定平面格的地面高度。 */
    void elevateByMinimum(int cell);

    /** @brief 构造时注入的只读配置；本类不修改它（见构造函数注释）。 */
    const PerFrameHeightConfig& config_;
  };

}  // namespace terrain_analysis
