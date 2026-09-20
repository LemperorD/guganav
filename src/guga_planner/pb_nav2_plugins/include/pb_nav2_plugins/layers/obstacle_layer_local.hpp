/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef PB_NAV2_PLUGINS__LAYERS__OBSTACLE_LAYER_LOCAL_HPP_
#define PB_NAV2_PLUGINS__LAYERS__OBSTACLE_LAYER_LOCAL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pb_nav2_costmap_2d
{

/**
 * @class ObstacleLayerLocal
 * @brief 接收激光与点云数据，填充到 2D 代价地图
 */
class ObstacleLayerLocal : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief 构造函数
   */
  ObstacleLayerLocal()
  {
    costmap_ = NULL;  // 这是父类 Costmap2D 中的 unsigned char* 成员
  }

  /**
   * @brief 析构函数
   */
  virtual ~ObstacleLayerLocal();
  /**
   * @brief 节点启动时该图层的初始化流程
   */
  virtual void onInitialize();
  /**
   * @brief 按本图层的更新范围扩展主代价地图的更新边界
   * @param robot_x 机器人位姿 X
   * @param robot_y 机器人位姿 Y
   * @param robot_yaw 机器人朝向
   * @param min_x 待更新窗口在地图坐标下的 X 最小值
   * @param min_y 待更新窗口在地图坐标下的 Y 最小值
   * @param max_x 待更新窗口在地图坐标下的 X 最大值
   * @param max_y 待更新窗口在地图坐标下的 Y 最大值
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);
  /**
   * @brief 更新窗口内主代价地图的代价值
   * @param master_grid 待更新的主代价地图栅格
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief 停用该图层
   */
  virtual void deactivate();

  /**
   * @brief 启用该图层
   */
  virtual void activate();

  /**
   * @brief 复位该代价地图
   */
  virtual void reset();

  /**
   * @brief 该图层是否需要处理清除操作
   */
  virtual bool isClearable() { return true; }

  /**
   * @brief 检测到参数变化时执行的回调
   * @param parameters 发生变化的参数列表
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief 触发观测缓冲的时间戳更新
   */
  void resetBuffersLastUpdated();

  /**
   * @brief 缓存 LaserScan 消息的回调
   * @param message 消息过滤器返回的消息
   * @param buffer 指向待更新观测缓冲的指针
   */
  void laserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

  /**
   * @brief 缓存 LaserScan 消息的回调，先把无效值 Inf 过滤成 range_max
   * @param message 消息过滤器返回的消息
   * @param buffer 指向待更新观测缓冲的指针
   */
  void laserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

  /**
   * @brief 缓存 PointCloud2 消息的回调
   * @param message 消息过滤器返回的消息
   * @param buffer 指向待更新观测缓冲的指针
   */
  void pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

  // 仅供测试使用
  void addStaticObservation(nav2_costmap_2d::Observation & obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);

protected:
  /**
   * @brief 取出用于标记障碍的观测
   * @param marking_observations 用于填入观测结果的向量引用
   * @return 所有观测缓冲都是最新的返回 true，否则返回 false
   */
  bool getMarkingObservations(
    std::vector<nav2_costmap_2d::Observation> & marking_observations) const;

  /**
   * @brief 取出用于清除自由空间的观测
   * @param clearing_observations 用于填入观测结果的向量引用
   * @return 所有观测缓冲都是最新的返回 true，否则返回 false
   */
  bool getClearingObservations(
    std::vector<nav2_costmap_2d::Observation> & clearing_observations) const;

  /**
   * @brief 依据单次观测清除自由空间
   * @param clearing_observation 用于射线追踪的观测
   * @param min_x 待更新窗口在地图坐标下的 X 最小值
   * @param min_y 待更新窗口在地图坐标下的 Y 最小值
   * @param max_x 待更新窗口在地图坐标下的 X 最大值
   * @param max_y 待更新窗口在地图坐标下的 Y 最大值
   */
  virtual void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation, double * min_x, double * min_y,
    double * max_x, double * max_y);

  /**
   * @brief 用射线追踪的结果更新窗口边界
   */
  void updateRaytraceBounds(
    double ox, double oy, double wx, double wy, double max_range, double min_range, double * min_x,
    double * min_y, double * max_x, double * max_y);

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  /**
   * @brief 清除机器人足迹范围内的图层信息
   */
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);

  std::string global_frame_;    ///< @brief 代价地图使用的全局坐标系
  double min_obstacle_height_;  ///< @brief 障碍物最小高度
  double max_obstacle_height_;  ///< @brief 障碍物最大高度

  /// @brief 用于把激光扫描投影成点云
  laser_geometry::LaserProjection projector_;
  /// @brief 用于观测消息的订阅者
  std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>>
    observation_subscribers_;
  /// @brief 用于确保每个传感器都有可用的坐标变换
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
  /// @brief 用于保存各传感器产生的观测
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> observation_buffers_;
  /// @brief 用于保存标记障碍所用的观测缓冲
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> marking_buffers_;
  /// @brief 用于保存清除障碍所用的观测缓冲
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> clearing_buffers_;

  /// @brief 动态参数回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // 仅供测试使用
  std::vector<nav2_costmap_2d::Observation> static_clearing_observations_;
  std::vector<nav2_costmap_2d::Observation> static_marking_observations_;

  bool rolling_window_;
  bool was_reset_;
  int combination_method_;
};

}  // namespace pb_nav2_costmap_2d

#endif  // PB_NAV2_PLUGINS__LAYERS__OBSTACLE_LAYER_LOCAL_HPP_
