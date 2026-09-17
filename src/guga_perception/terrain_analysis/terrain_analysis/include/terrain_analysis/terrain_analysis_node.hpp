#pragma once

#include "terrain_analysis/core/terrain_processor.hpp"
#include "terrain_analysis/core/terrain_voxel_map.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace terrain_analysis {
  /**
   * @brief terrain_analysis ROS2 节点封装。
   *
   * 只负责 ROS 层的接线：声明参数、订阅 odom/点云、按固定周期驱动
   * TerrainProcessor 并把结果发布为 terrain_map。算法与状态都在 Processor 内。
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
     * @brief 驱动一次处理：若有无新点云则跑管线并发布。
     * @return ROS 上下文仍运行时返回 true，否则返回 false。
     */
    bool processOnce();

    /** @brief 获取最近一次生成的带高度点云。 */
    [[nodiscard]] const pcl::PointCloud<pcl::PointXYZI>& terrainCloudElev()
        const {
      return processor_.terrainCloudElev();
    }
    /** @brief 获取可修改的算法配置，主要用于测试和节点初始化。 */
    [[nodiscard]] TerrainConfig& config() noexcept {
      return processor_.config();
    }
    /** @brief 获取处理器，供白盒测试驱动。 */
    [[nodiscard]] TerrainProcessor& processor() noexcept {
      return processor_;
    }
    /** @brief 获取只读处理器。 */
    [[nodiscard]] const TerrainProcessor& processor() const noexcept {
      return processor_;
    }
    /** @brief 跨帧持久的体素地图；由节点持有并逐帧分发给处理器。 */
    [[nodiscard]] TerrainVoxelMap& voxelMap() noexcept {
      return voxel_map_;
    }

  private:
    /** @brief 将内部输出点云转换为 ROS 消息并发布。 */
    void publishPointCloud();

    /** @brief 跨帧持久的体素地图：节点是它的属主，处理器按帧接收。 */
    TerrainVoxelMap voxel_map_;
    TerrainProcessor processor_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        sub_laser_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_terrain_map_;
    rclcpp::TimerBase::SharedPtr timer_;
  };
}  // namespace terrain_analysis
