/**
 * @file PriorMapManager.h
 * @brief 先验地图的累积、写盘与重载.
 *
 * 由 GICP 状态驱动, 负责 PCD 路径生成、原子写盘、增量更新与重载请求.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "point_lio/OdomProcessor.h"
#include "point_lio/core/common_lib.h"
#include "point_lio/core/parameters.h"

/// 先验地图的累积、写盘与重载, 由 GICP 状态驱动.
class PriorMapManager {
public:
  void configure(const PublishParams& params, const OutputFrameParams& out,
                 OdomProcessor* projector, rclcpp::Node* node,
                 rclcpp::CallbackGroup::SharedPtr group);

  /// 生成第 index 个输出文件的路径; index 为 0 时按时间戳命名.
  [[nodiscard]] std::string outputPath(int index) const;

  /// 原子写入 PCD: 先写临时文件再替换, 避免留下半个文件.
  [[nodiscard]] bool writePcd(const std::string& path,
                              const PointCloudXYZI& cloud) const;

  /// 主循环每帧调用: 累积点云, 并在满足条件时写盘.
  void update(const PointCloudXYZI& world_cloud);

  /// 进程退出前把未写完的缓冲落盘.
  void savePending();

  /// 当前是否处于累积状态.
  [[nodiscard]] bool active() const;

private:
  void onGicpState(uint8_t state);
  void arm();
  void backupPriorMap();
  bool mergeMapAndWrite();
  void saveOnLost();
  void requestReload();

  const PublishParams* params_{nullptr};
  const OutputFrameParams* out_{nullptr};
  OdomProcessor* projector_{nullptr};
  rclcpp::Node* node_{nullptr};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_client_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

  PointCloudXYZI::Ptr buffer_{std::make_shared<PointCloudXYZI>()};
  int scan_count_{0};
  int index_{0};

  std::atomic<int> gicp_state_{0};
  std::atomic<bool> armed_{false};
  std::atomic<bool> save_requested_{false};
  bool save_on_lost_done_{false};
  bool first_rebuild_done_{false};
  bool suspect_tracking_{false};
  double suspect_accum_sec_{0.0};
  rclcpp::Time suspect_mark_;
  bool updated_this_suspect_{false};
};
