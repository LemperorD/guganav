#pragma once

#include "terrain_analysis_ext/core/state.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * @brief terrain_analysis_ext ROS2 节点封装。
 *
 * 订阅 terrain_map 和 lidar_odometry，筛选车辆周围指定半径内的地形点，
 * 并发布 terrain_map_ext。
 */
class TerrainAnalysisExtNode {
public:
  /** @brief 构造节点封装，声明参数并创建 ROS 通信接口。 */
  explicit TerrainAnalysisExtNode(rclcpp::Node* node);
  /** @brief 默认析构，释放 ROS 句柄和运行时状态。 */
  ~TerrainAnalysisExtNode() = default;

  TerrainAnalysisExtNode(const TerrainAnalysisExtNode&) = delete;
  TerrainAnalysisExtNode& operator=(const TerrainAnalysisExtNode&) = delete;
  TerrainAnalysisExtNode(TerrainAnalysisExtNode&&) = delete;
  TerrainAnalysisExtNode& operator=(TerrainAnalysisExtNode&&) = delete;

  /**
   * @brief 处理一帧待处理地形点云并发布扩展地图。
   * @return ROS 上下文仍运行时返回 true，否则返回 false。
   */
  bool processOnce();

private:
  /** @brief 车辆周围局部地形地图的保留半径，单位为米。 */
  double local_terrain_map_radius_ = 4.0;
  /** @brief 将内部输出点云转换为 ROS 消息并发布。 */
  void publishPointCloud();

  TerrainExtState state_;

  rclcpp::Node* node_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_local_terrain_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_map_;

  rclcpp::TimerBase::SharedPtr timer_;
};
