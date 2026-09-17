#pragma once

#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/state.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>

namespace terrain_analysis {

  /**
   * @brief 地形分析处理器：拥有算法状态、对外只暴露"一件事"一个入口。
   *
   * 设计意图（可见性契约）：
   *   - 管线各阶段（voxelizeTerrain / updateTerrainVoxels /
   * estimateTerrainGround / …）是**实现细节**， 一律
   * private，可自由重构签名而不影响任何外部调用方。
   *   - 外部只能通过三个语义入口驱动：ingestOdometry / ingestLaserCloud /
   *     run，并读取 terrainCloudElev()。
   *   - 需要白盒验证各阶段的测试通过 `friend` 显式获得访问权（见下），
   *     刻意不把这些阶段提升为公开 API —— 否则内部编排序会固化成对外契约。
   *
   * 线程模型：本类**不做同步**。调用方必须保证 ingest* 与 run 不并发执行
   * （当前由节点在单线程执行器中串行调用满足）。
   */
  class TerrainProcessor {
  public:
    TerrainProcessor() = default;

    TerrainProcessor(const TerrainProcessor&) = delete;
    TerrainProcessor& operator=(const TerrainProcessor&) = delete;
    TerrainProcessor(TerrainProcessor&&) = delete;
    TerrainProcessor& operator=(TerrainProcessor&&) = delete;

    /**
     * @brief 接收里程计位姿，更新雷达位置与姿态三角函数缓存。
     *
     * 注意：传入的是**雷达**位姿（`loam_interface` 发布的 `lidar_odometry`
     * child frame 为 `front_mid360`），不是车体位姿；姿态中含雷达安装倾角。
     * @param x 雷达在 odom 坐标系下的 x 位置。
     * @param y 雷达在 odom 坐标系下的 y 位置。
     * @param z 雷达在 odom 坐标系下的 z 位置。
     * @param roll 雷达 roll 角，单位为弧度。
     * @param pitch 雷达 pitch 角，单位为弧度。
     * @param yaw 雷达 yaw 角，单位为弧度。
     */
    void ingestOdometry(double x, double y, double z, double roll, double pitch,
                        double yaw);

    /**
     * @brief 接收当前帧激光点云并执行范围、相对高度预过滤。
     * @param cloud 输入点云，坐标应位于 odom 坐标系。
     * @param timestamp_sec 点云时间戳，单位为秒。
     */
    void ingestLaserCloud(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
        double timestamp_sec);

    /**
     * @brief 执行一次完整地形分析管线。
     *
     * 仅在存在待处理点云时才会实际运算；无新点云时直接返回，
     * 输出点云保持上一次结果。返回前发布所需的输出已就绪。
     */
    void run();

    /** @brief 是否存在尚未处理的点云帧。 */
    [[nodiscard]] bool hasPendingCloud() const noexcept {
      return state_.new_laser_cloud;
    }

    /** @brief 最近一帧点云的时间戳，单位为秒（用于给输出打时间戳）。 */
    [[nodiscard]] double laserCloudTime() const noexcept {
      return state_.laser_cloud_time;
    }

    /** @brief 最近一次生成的带离地高度点云（intensity 为离地高度）。 */
    [[nodiscard]] const pcl::PointCloud<pcl::PointXYZI>& terrainCloudElev()
        const noexcept {
      return *state_.terrain_cloud_elev;
    }

    /** @brief 可修改的算法配置。 */
    [[nodiscard]] TerrainConfig& config() noexcept {
      return config_;
    }
    /** @brief 只读的算法配置。 */
    [[nodiscard]] const TerrainConfig& config() const noexcept {
      return config_;
    }

  private:
    // ── 私有类型 ──
    /** @brief 网格轴向（仅 shiftGrid 搬运方向需要区分轴）。 */
    enum class Axis : uint8_t { AXIS_X, AXIS_Y };
    /** @brief 网格内容搬运方向：沿索引增大 / 减小，即世界坐标正向 / 负向。 */
    enum class ShiftDirection : uint8_t {
      TOWARD_NEGATIVE,
      TOWARD_POSITIVE,
    };
    /** @brief 要换算到的网格种类。 */
    enum class VoxelGrid : uint8_t {
      TERRAIN,  ///< 跨帧累积的 terrain voxel 网格。
      PLANAR,   ///< 逐帧重建的 planar voxel 网格。
    };
    /** @brief 平面点所属的网格下标；越界时 valid 为 false，row/col 无意义。 */
    struct GridIndex {
      int row = 0;
      int col = 0;
      bool valid = false;
    };
    /** @brief 点转换到传感器系后的坐标。 */
    struct SensorPoint {
      double x;
      double y;
      double z;
    };

    /**
     * @brief 将整张 terrain voxel 网格沿指定轴搬运一格，腾出的新格清空。
     * @param axis 搬运轴向。
     * @param direction 搬运方向。
     */
    void shiftGrid(Axis axis, ShiftDirection direction);

    // ── 内部判定与运算（读写 config_/state_，故为成员而非自由函数）──
    /** @brief 该点相对雷达的水平距离。 */
    [[nodiscard]] double horizontalDistanceTo(double px, double py) const;
    /** @brief 该 terrain voxel 本轮是否需要降采样/衰减重建。 */
    [[nodiscard]] bool shouldPruneTerrainVoxel(int cell) const;
    /**
     * @brief 该点是否应保留在该 terrain voxel 中。
     * @param relative_z 点相对雷达的高度。
     * @param distance 点相对雷达的水平距离。
     * @param point_time 点的采集时刻（相对首帧，单位秒）。
     */
    [[nodiscard]] bool keepTerrainVoxelPoint(double relative_z, double distance,
                                             double point_time) const;
    /** @brief 把相对雷达的坐标变换到传感器坐标系。 */
    [[nodiscard]] SensorPoint transformToSensorFrame(double x, double y,
                                                     double z) const;
    /** @brief 清空 planar voxel 的地面候选、高程估计与动态障碍计数。 */
    void resetPlanarVoxels();
    /**
     * @brief 把一个高度值加入指定 planar voxel 及其 3×3 邻域的候选集中。
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
    /** @brief 用分位数估计指定 planar voxel 的地面高度。 */
    void elevateByQuantile(int cell);
    /** @brief 用最低点估计指定 planar voxel 的地面高度。 */
    void elevateByMinimum(int cell);

    // ── 管线阶段（实现细节，见类注释的可见性契约）──
    void rolloverTerrainVoxels();
    void voxelizeTerrain();
    void updateTerrainVoxels();
    void collectTerrainCloud();
    void estimateTerrainGround();
    void detectDynamicObstacles();
    void filterDynamicObstaclePoints();
    void computePlanarElevation();
    void computeHeightMap();

    // ── 无状态工具（不依赖 config_/state_，故为静态成员，置于末尾）──
    /**
     * @brief 把一个平面点换算成指定网格的行列下标，越界时返回 invalid。
     *
     * 雷达位置由 state_ 读取，不由调用方传入——网格索引的基准始终是"当前
     * 雷达位置"，避免调用点各自快照位姿造成同帧内基准不一致。
     * @param grid 目标网格种类。
     * @param x 点在 odom 坐标系下的 x。
     * @param y 点在 odom 坐标系下的 y。
     * @return 行列下标；越界时 GridIndex::valid 为 false。
     */
    [[nodiscard]] GridIndex voxelIndexOf(VoxelGrid grid, double x,
                                         double y) const;

    TerrainConfig config_;
    TerrainState state_;

    // 白盒测试需要逐阶段驱动与检查内部状态；仅授予本包测试 fixture，
    // 不对外开放（新增测试如需访问，在此显式追加）。
    friend class AlgorithmTest;
    friend class StateIngestTest;
    friend class TerrainAnalysisTest;
    friend class StageRunner;
  };

}  // namespace terrain_analysis
