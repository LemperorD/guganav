#pragma once

#include "terrain_analysis/core/config.hpp"
#include "terrain_analysis/core/state.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

/**
 * @brief terrain_analysis ROS2 节点封装。
 *
 * 负责声明算法参数、订阅里程计和注册点云、接收清除请求，并以固定周期
 * 驱动 terrain_analysis::algorithm 管线后发布 terrain_map。
 */
class TerrainAnalysis {
public:
  /** @brief 构造节点封装，声明参数、创建订阅/发布器和处理定时器。 */
  explicit TerrainAnalysis(rclcpp::Node* node);
  /** @brief 默认析构，释放 ROS 句柄和算法状态。 */
  ~TerrainAnalysis() = default;

  TerrainAnalysis(const TerrainAnalysis&) = delete;
  TerrainAnalysis& operator=(const TerrainAnalysis&) = delete;
  TerrainAnalysis(TerrainAnalysis&&) = delete;
  TerrainAnalysis& operator=(TerrainAnalysis&&) = delete;

  /**
   * @brief 处理一帧待处理点云并发布结果。
   * @return ROS 上下文仍运行时返回 true，否则返回 false。
   */
  bool processOnce();

  /** @brief 获取最近一次生成的带高度点云。 */
  [[nodiscard]] const pcl::PointCloud<pcl::PointXYZI>& terrainCloudElev()
      const {
    return *state_.terrain_cloud_elev;
  }
  /** @brief 获取可修改的算法配置，主要用于测试和节点初始化。 */
  [[nodiscard]] TerrainConfig& config() {
    return config_;
  }
  /** @brief 获取可修改的算法状态，主要用于测试和算法驱动。 */
  [[nodiscard]] TerrainState& state() {
    return state_;
  }
  /** @brief 获取只读算法状态。 */
  [[nodiscard]] const TerrainState& state() const {
    return state_;
  }

private:
  /** @brief 将内部输出点云转换为 ROS 消息并发布。 */
  void publishPointCloud();

  rclcpp::Node* node_;
  TerrainConfig config_;
  TerrainState state_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_laser_cloud_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_clearing_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_map_;
  rclcpp::TimerBase::SharedPtr timer_;
};
