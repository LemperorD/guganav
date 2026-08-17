#pragma once

#include "terrain_analysis_ext/core/state.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <vector>

namespace terrain_analysis_ext {

  /**
   * @brief terrain_analysis_ext ROS2 节点（rclcpp 组件）。
   *
   * 订阅 terrain_map 和 lidar_odometry，筛选车辆周围指定半径内的地形点，
   * 并发布 terrain_map_ext。
   *
   * 内置延迟插桩（组件化前后对比用）：记录 terrain_map 回调到达时刻到
   * terrain_map_ext 发布时刻的耗时，每 200 帧输出 p50/p95/max 统计日志
   * （前缀 `[benchmark]`，供脚本抓取）。
   */
  class TerrainAnalysisExtNode : public rclcpp::Node {
  public:
    /** @brief 以组件方式构造节点，声明参数并创建 ROS 通信接口。 */
    explicit TerrainAnalysisExtNode(const rclcpp::NodeOptions& options);
    /** @brief 默认析构。 */
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
    /** @brief 每多少帧输出一次延迟统计。 */
    static constexpr size_t kBenchmarkWindow = 200;
    /** @brief 将内部输出点云转换为 ROS 消息并发布。 */
    void publishPointCloud();
    /** @brief 输出最近窗口的接收→发布延迟统计（p50/p95/max）。 */
    void printLatencyStats();

    /** @brief 车辆周围局部地形地图的保留半径，单位为米。 */
    double local_terrain_map_radius_ = 4.0;

    TerrainExtState state_;

    /** @brief 最近一帧 terrain_map 回调到达时刻（wall clock）。 */
    std::chrono::steady_clock::time_point receive_wall_;
    /** @brief 最近窗口内的接收→发布延迟，单位为毫秒。 */
    std::vector<double> latency_ms_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        sub_local_terrain_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_terrain_map_;

    rclcpp::TimerBase::SharedPtr timer_;
  };

}  // namespace terrain_analysis_ext
