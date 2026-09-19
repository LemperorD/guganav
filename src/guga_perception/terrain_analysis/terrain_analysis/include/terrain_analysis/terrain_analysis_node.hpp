#pragma once

#include "terrain_analysis/per_frame_height_map.hpp"
#include "terrain_analysis/persistent_voxel_map.hpp"
#include "guga_common/geometry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace terrain_analysis {
  /**
   * @brief terrain_analysis ROS2 节点封装。
   *
   * 只负责 ROS 层的接线与**逐帧数据分发**：声明参数、订阅里程计与点云、把接收
   * 与累积交给跨帧持续的 PersistentVoxelMap、把采集结果交给逐帧的
   * PerFrameHeightMap、发布 terrain_map。两半各自的算法状态都在它们自己内部。
   */
  class TerrainAnalysis : public rclcpp::Node {
  public:
    /** @brief 构造节点封装，声明参数、创建订阅/发布器和处理定时器。 */
    explicit TerrainAnalysis(const rclcpp::NodeOptions& options);
    /** @brief 默认析构，释放 ROS 句柄和算法状态。 */
    ~TerrainAnalysis() = default;

    TerrainAnalysis(const TerrainAnalysis&) = delete;
    TerrainAnalysis& operator=(const TerrainAnalysis&) = delete;
    TerrainAnalysis(TerrainAnalysis&&) = delete;
    TerrainAnalysis& operator=(TerrainAnalysis&&) = delete;

    /**
     * @brief 驱动一次处理：若有无新点云则跑两半管线并发布。
     * @return ROS 上下文仍运行时返回 true，否则返回 false。
     */
    bool processOnce();

    /**
     * @brief 同步跑一帧：收下这帧点云与雷达位置，跑完整条管线并发布。
     *
     * 与两条订阅等价，只是不必经过 ROS 话题——便于进程内嵌入、回放与测试。
     * 定时器驱动的生产方式仍走订阅 + processOnce()。
     * @param cloud 本帧点云，坐标位于 odom 坐标系。
     * @param lidar_position 雷达在 odom 下的位置（不是车体位置）。
     * @param timestamp_sec 本帧时间戳，单位为秒。
     * @return ROS 上下文仍运行时返回 true，否则返回 false。
     */
    bool processFrame(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      const guga_common::Point3d& lidar_position,
                      double timestamp_sec);

    /** @brief 获取最近一次生成的障碍点云。 */
    [[nodiscard]] const pcl::PointCloud<pcl::PointXYZI>& obstacleCloud() const {
      return per_frame_height_map_.obstacleCloud();
    }

  private:
    /** @brief 将内部输出点云转换为 ROS 消息并发布。 */
    void publishPointCloud();

    /** @brief 记下本帧的雷达位置与时刻，把点云交给前半段（两条订阅与
     * processFrame 共用这一处）。 */
    void ingestFrame(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                     const guga_common::Point3d& lidar_position,
                     double timestamp_sec);

    /** @brief 声明前半段读取的 ROS 参数并返回填好的配置。 */
    PersistentVoxelConfig declareVoxelConfig();
    /**
     * @brief 声明后半段读取的 ROS
     * 参数并返回填好的配置（含启动时的高度带日志）。
     * @param min_relative_z 两半共用的 minRelZ；它只声明一次（在前半段那侧），
     *        这里直接取已声明的值，避免同一个参数被声明两次。
     */
    PerFrameHeightConfig declareHeightConfig(double min_relative_z);

    // 参数由节点声明并持有；两半只保存它们的常量引用，因此这两份必须声明在两半
    // 之前——引用既要活得过两半，析构顺序也要相反（两半先销毁，配置后销毁）。
    /** @brief 前半段读取的参数。 */
    PersistentVoxelConfig voxel_config_;
    /** @brief 后半段读取的参数。 */
    PerFrameHeightConfig height_config_;
    /** @brief 前半段：接收本帧输入并维护跨帧体素地图。 */
    PersistentVoxelMap persistent_voxel_map_;
    /** @brief 后半段：由采集点云估计地面并生成障碍输出。 */
    PerFrameHeightMap per_frame_height_map_;
    /** @brief 两半之间的交接数据（采集点云）。 */
    pcl::PointCloud<pcl::PointXYZI>::Ptr collected_cloud_ =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    /** @brief 最近一帧的雷达位置（里程计回调写入，再分发给两半）。 */
    guga_common::Point3d lidar_position_;
    /** @brief 最近一帧的时间戳，单位为秒（用于给输出打时间戳）。 */
    double last_stamp_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        sub_laser_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_terrain_map_;
    rclcpp::TimerBase::SharedPtr timer_;
  };
}  // namespace terrain_analysis
