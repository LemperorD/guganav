/**
 * @file FramePublisher.h
 * @brief camera_init 系点云发布.
 *
 * 把世界系点云和初始化地图转成消息后发布, 不涉及坐标变换.
 */

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"

/// 点云发布的封装: 只负责把点云转成消息并发布.
class FramePublisher {
public:
  void configure(rclcpp::Node* node, const PublishParams& params);

  void publishWorld(const PointCloudXYZI& cloud,
                    const rclcpp::Time& stamp) const;
  void publishMap(const sensor_msgs::msg::PointCloud2& msg) const;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  const PublishParams* params_{nullptr};
};
