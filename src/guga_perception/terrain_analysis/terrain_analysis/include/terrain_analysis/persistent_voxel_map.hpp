#pragma once

#include "terrain_analysis/config.hpp"
#include "terrain_analysis/grid.hpp"
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
   * 两半的分界是**寿命**：本类跨帧持续，是管线里唯一保留帧间状态的一侧；后半段
   * PerFrameHeightMap 每帧重建、帧间不存任何东西。
   *
   * 持有两样东西：
   *   - 跨帧保留的点云（管线里唯一一份），随雷达移动滚动；
   *   - 本帧输入：裁剪后的点云、雷达位置、帧时刻与首帧时刻。
   *
   * 配置只在构造时以常量引用注入，之后本类不再接受任何配置（也就没有"可写配置"
   * 这种入口）；所有者可以改自己那份，本类下一次调用就会用到新的值。
   *
   * 裁剪放在这里而不是调用方，原因是那两条判据（高度带、接收半径）本来就在服务
   * 这张网格的接收范围：接收半径就是网格宽度，高度带与 rebuild 的 keepPoint
   * 同源。
   *
   * 一帧的流程：ingest() 收帧 → update()（滚动 / 归格 / 重建三步）→
   * collectCloud() 取窗口。update() 用 ingest() 记下的同一个锚点完成三步，
   * 调用方无法把不同的锚点混进同一帧。
   *
   * 对外只有上面这些入口；逐阶段方法与内部数据放在 protected 的"接缝"区，
   * 需要白盒验证的测试用派生类把它们提升为公有（见 test/test_doubles.hpp），
   * 生产头文件里不出现测试类名。
   *
   * 线程模型：本类不做同步，调用方必须保证 ingest 与 update 不并发执行（当前由
   * 节点在单线程执行器中串行调用满足）。
   */
  class PersistentVoxelMap {
  public:
    /** @brief 一格累积点云。 */
    using Cell = pcl::PointCloud<pcl::PointXYZI>;

    /**
     * @brief 以配置构造：只保存常量引用，既不拷贝也不修改它。
     *
     * 引用而非副本，是为了让所有者（节点、测试 fixture、测量工具）改动自己那份
     * 配置后，本类下一次调用就能看到，不必重建对象。因此：
     *   - config 必须比本对象活得更久（所有者先声明、后销毁）；
     *   - 右值构造被显式删除，避免绑定到临时量；
     *   - 所有者改动配置的时机必须是"本对象不在运行中"——本类不做任何同步，
     *     运行中被改会读到半新半旧的配置。
     */
    explicit PersistentVoxelMap(const PersistentVoxelConfig& config) noexcept
        : config_(config) {
    }
    /** @brief 禁止绑定临时配置：引用会立即悬垂。 */
    PersistentVoxelMap(PersistentVoxelConfig&&) = delete;

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

    /** @brief 最近一帧的雷达位置（odom 下）；后半段以它作平面网格锚点。 */
    [[nodiscard]] const guga_common::Point3d& lidarPosition() const noexcept {
      return lidar_;
    }

    [[nodiscard]] bool hasPendingFrame() const noexcept {
      return frame_pending_;
    }

    /**
     * @brief 本帧维护：滚动窗口 → 本帧点云归格 → 逐格重建。
     *
     * 使用 ingest() 记下的雷达位置与时刻，以及构造时注入的配置。
     */
    void update();

    /**
     * @brief 采集地图中央窗口内的累积点云。
     *
     * 取以雷达为中心的 11x11 格（约 ±5.5 m）——窗口大小是本类的策略，因此由本类
     * 决定，而不是让调用方记住；拼接本身是"格 → 点云"的通用换算，见
     * grid_utils.hpp。
     * @param out 输出容器，会被清空后填入采集结果。
     */
    void collectCloud(Cell& out) const;

    /**
     * @brief 裁剪后的本帧点云（intensity 为观测时刻，相对首帧的秒数）。
     *
     * 节点用它生成两份当帧输出（见 PerFrameHeightMap::computeFrameOutputs）：
     * 累计云代表"感知历史"，而射线清除需要的是本帧这一份观测。
     */
    [[nodiscard]] const Cell& frameCloud() const noexcept {
      return *frame_cloud_;
    }

  private:
    static constexpr int EXTRACT_HALF_WINDOW = 5;

  protected:
    // ── 接缝：本类内部编排与数据。生产代码不用，白盒测试用派生类把它们提升为
    // 公有（见 test/test_doubles.hpp）——与 nav2_mppi_controller 的做法一致：
    // 生产头文件里不出现测试类名，封口也不靠 friend 名单维护。 ──
    // 三个阶段都作用在"本帧"上：锚点、本帧点云与时刻已由 ingest() 记在成员里，
    // 因此它们不收参数——地图持有这一帧，成员函数直接读它。
    /** @brief 滚动网格，维持以雷达为中心的窗口（update 的第一步）。 */
    void rollover();

    /** @brief 把本帧点云按位置分配到体素格（update 的第二步）。 */
    void addFrame();

    /**
     * @brief 逐格重建：按叶保留最新观测点，并做高度带与年龄过滤（update
     * 的第三步）。
     *
     * 每个叶（水平 scan_voxel_size、垂直 scan_voxel_size_z）只保留观测时刻最新
     * 的那一个点，于是"有新点即刷新、无新点才判年龄"不需要额外状态：代表点自带
     * 的时刻就是该叶的 last_seen。
     */
    /** @brief 第三步，年龄按 elapsedSeconds() 判定。 */
    void rebuildGrids();

    /** @brief 最近一帧的时间戳（秒）。 */
    [[nodiscard]] double timestamp() const noexcept {
      return time_;
    }

    /** @brief 最近一帧相对首帧的秒数。 */
    [[nodiscard]] double elapsedSeconds() const noexcept {
      return time_ - init_time_;
    }

    [[nodiscard]] const std::array<Cell::Ptr, PersistentVoxelGrid::NUM>& cells()
        const noexcept {
      return cloud_;
    }
    [[nodiscard]] std::array<Cell::Ptr, PersistentVoxelGrid::NUM>&
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

    /**
     * @brief 该点是否落在接收带内。**雷达系**：高度是相对雷达的偏移，带宽随水平
     * 距离放宽；两侧都是严格不等，等于边界的点排除。
     *
     * `ingest` 的裁剪与 `rebuild` 的保留用的是同一个带，判据只此一处——两处各写
     * 一遍正是"改了一处忘了另一处"的来源。
     */
    [[nodiscard]] bool insideReceiveBand(double z_rel_lidar, double distance);

    /**
     * @brief 该点是否应保留在该体素格中：接收带内，且年龄未达 `decay_time`。
     *
     * 年龄一律按观测时刻算，不再有"近处豁免"——那条规则用距离去猜"看不见了"，
     * 会把已被搬走的近处残影一并留住，实车配置里本来就是 0（等于关闭）。
     * @param z_rel_lidar 相对雷达的高度（**雷达系**）。
     * @param distance 水平距离（与参考系无关）。
     * @param point_time 该点的观测时刻（相对首帧的秒数），现由 intensity 携带。
     */
    [[nodiscard]] bool keepPoint(double z_rel_lidar, double distance,
                                 double point_time, double now_elapsed);

    std::array<Cell::Ptr, PersistentVoxelGrid::NUM> cloud_ = makeCells();
    // 本帧输入：由 ingest 写入，update 消费。
    Cell::Ptr frame_cloud_ = std::make_shared<Cell>();
    guga_common::Point3d lidar_;
    double time_ = 0.0;
    double init_time_ = 0.0;

  private:
    /** @brief 融合叶键：x/y 与 z 使用不同叶宽（垂直更细）。 */
    [[nodiscard]] uint64_t leafKey(double x, double y, double z);

    /** @brief 滚动时要搬运的轴。 */
    enum class ShiftAxis { X, Y };

    /** @brief 内容搬向哪一侧的下标：POSITIVE 指下标增大的一侧。 */
    enum class ShiftDirection { NEGATIVE, POSITIVE };

    /**
     * @brief 把整张网格沿指定轴搬运一格，腾出的新格清空。
     *
     * 轴与 gridIndex 的约定必须一致（x 在列上、y
     * 在行上）——写反过一次，见实现处的
     * 注释。方向说的是**内容**搬向哪一侧的下标：车向 +x 移动一格时，锚点 x
     * 增大， 同一世界点的列下标要减 1，因此内容搬向下标更小的一侧。
     */
    void shift(ShiftAxis axis, ShiftDirection positive);

    /** @brief 构造时注入的只读配置；本类不修改它（见构造函数注释）。 */
    const PersistentVoxelConfig& config_;
    int shift_x_ = 0;
    int shift_y_ = 0;
    bool inited_ = false;
    bool frame_pending_ = false;

    [[nodiscard]] static std::array<Cell::Ptr, PersistentVoxelGrid::NUM>
    makeCells();
  };

}  // namespace terrain_analysis
