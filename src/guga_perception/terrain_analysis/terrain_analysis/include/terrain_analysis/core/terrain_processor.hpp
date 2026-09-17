#pragma once

#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/grid_lookup.hpp"
#include "terrain_analysis/core/terrain_voxel_map.hpp"
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
    /** @brief 点转换到传感器系后的坐标。 */
    struct SensorPoint {
      double x;
      double y;
      double z;
    };

    // ── 内部判定与运算（读写 config_/state_，故为成员而非自由函数）──
    /** @brief 该点相对雷达的水平距离。 */
    [[nodiscard]] double horizontalDistanceTo(double px, double py) const;
    /** @brief 把相对雷达的坐标变换到传感器坐标系。 */
    [[nodiscard]] SensorPoint transformToSensorFrame(double x, double y,
                                                     double z) const;
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
    // 体素地图的三个阶段（rollover / voxelize / update）已提取为包内自由函数，
    // 见 terrain_voxel_map.hpp；这里只保留编排与其余阶段。
    void collectTerrainCloud();
    void estimateTerrainGround();
    void detectDynamicObstacles();
    void filterDynamicObstaclePoints();
    void computePlanarElevation();
    void computeHeightMap();

    /** @brief 跨帧持久的体素地图（体素阶段的属主，见 terrain_voxel_map.hpp）。
     */
    TerrainVoxelMap voxel_map_;

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
