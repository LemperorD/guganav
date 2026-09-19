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
   * computeFrameOutputs 是第二条入口，把**本帧**点云过滤成两份当帧输出（供代价
   * 地图分别做标记与清除），地面场直接复用上面那条流水线算好的 voxel_elev_，
   * 因此必须在 compute() 之后调用。
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

    /**
     * @brief 由本帧点云生成两份当帧输出：障碍点云与回波点云。
     *
     * 与 compute() 产出的累计障碍云有三处不同，都是"当帧"这条语义要求的：
     *   -
     * 输入是本帧点云（PersistentVoxelMap::frameCloud），不是采集到的累计云；
     *   - 不做逐格点数门限：min_block_point_num 是按累计云的逐格点数定的，单帧
     *     每格点数远小于它，套用会几乎没有输出；
     *   - 同时产出两份。障碍点云再套一层输出带，供代价地图标记；回波点云只保留
     *     地面地板与穿透地板这两条筛选，含地面回波——清除射线需要的是"该方向上
     *     有一次回波"，地面回波正是"射线路径为空"的证据，它不必是障碍点。
     *
     * 两份输出都只保留平面网格窗口内的点（与累计输出同为约 5 m 半径），
     * intensity 都是距局部地面的高度 h。
     * @param frame_cloud 本帧点云，坐标位于 odom 坐标系。
     * @param lidar_position 雷达在 odom 下的位置（不是车体位置）。
     */
    void computeFrameOutputs(const Cell& frame_cloud,
                             const guga_common::Point3d& lidar_position);

    /** @brief 最近一次生成的当帧障碍点云（intensity 为距局部地面的高度）。 */
    [[nodiscard]] const Cell& frameObstacleCloud() const noexcept {
      return *frame_obstacle_cloud_;
    }

    /** @brief 最近一次生成的当帧回波点云（含地面回波，intensity
     * 为距局部地面的高度）。 */
    [[nodiscard]] const Cell& frameReturnCloud() const noexcept {
      return *frame_return_cloud_;
    }

  protected:
    // ── 接缝：三段阶段与网格数据。生产代码不用，白盒测试用派生类提升为公有
    // （见 test/test_doubles.hpp），生产头文件里不出现测试类名。 ──
    /**
     * @brief 该点是否在地面候选的入门地板之上。**odom 绝对高度**：地面在 odom
     * 里 大体水平，这条地板用来挡住远低于地面的穿透点，也是本阶段唯一的入门筛选
     * （把候选筛到地面系需要先有地面估计，见 README 的"两遍法"）。
     *
     * 严格不等：等于地板的点排除。
     */
    [[nodiscard]] static bool aboveGroundFloor(
        double z_odom, const PerFrameHeightConfig& config);

    /**
     * @brief 该点是否在障碍输出带内。**地面系**：h 是距局部地面的高度。
     *
     * 上界用带符号的 h（低于地面的坑不占净空），死区下界用 h 的绝对值——
     * `considerDrop` 打开时凹坑也算障碍。这个不对称是现状，语义待产品确认。
     */
    [[nodiscard]] static bool insideOutputBand(
        double h_ground, const PerFrameHeightConfig& config);

    /**
     * @brief
     * 该点是否在穿透点地板之上。**雷达系**：挡的是"远低于雷达"的穿透回波，
     * 与地形无关，因此留在雷达系；严格不等，等于地板的点排除。
     */
    [[nodiscard]] static bool abovePenetrationFloor(
        double z_rel_lidar, const PerFrameHeightConfig& config);

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
    Cell::Ptr obstacle_cloud_ = std::make_shared<Cell>();
    /** @brief 当帧输出：带内的障碍点，供代价地图标记。 */
    Cell::Ptr frame_obstacle_cloud_ = std::make_shared<Cell>();
    /** @brief 当帧输出：全部有效回波（含地面回波），供代价地图射线清除。 */
    Cell::Ptr frame_return_cloud_ = std::make_shared<Cell>();

  private:
    void elevateByQuantile(int cell);
    void elevateByMinimum(int cell);

    /** @brief 构造时注入的只读配置；本类不修改它（见构造函数注释）。 */
    const PerFrameHeightConfig& config_;
  };

}  // namespace terrain_analysis
