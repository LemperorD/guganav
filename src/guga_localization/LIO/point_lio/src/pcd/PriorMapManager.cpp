#include "point_lio/pcd/PriorMapManager.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

void PriorMapManager::configure(const PublishParams& params,
                                const OutputFrameParams& out,
                                OdomProcessor* projector, rclcpp::Node* node,
                                rclcpp::CallbackGroup::SharedPtr group) {
  params_ = &params;
  out_ = &out;
  projector_ = projector;
  node_ = node;
  if (!params.pcd_save_reload_service.empty()) {
    reload_client_ = node->create_client<std_srvs::srv::Trigger>(
        params.pcd_save_reload_service);
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  if (!params.pcd_save_state_topic.empty()) {
    state_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
        params.pcd_save_state_topic, rclcpp::QoS(1),
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          onGicpState(msg->data);
        },
        options);
  } else if (params.pcd_save_require_lost
             && !params.pcd_save_trigger_topic.empty()) {
    trigger_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        params.pcd_save_trigger_topic, rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            arm();
          }
        },
        options);
  }
}

bool PriorMapManager::active() const {
  if (!params_->pcd_save_enabled) {
    return false;
  }
  if (!params_->pcd_save_state_topic.empty()) {
    // 2=SUSPECT 3=LOST: 怀疑丢失后开始累积, 真正丢失时才写盘
    const int state = gicp_state_.load();
    return state == 2 || state == 3;
  }
  return !params_->pcd_save_require_lost || armed_.load();
}

void PriorMapManager::onGicpState(uint8_t state) {
  gicp_state_ = state;
  switch (state) {
    case 1:  // FINE: 暂停累积, 不清空缓存, 下次 SUSPECT 继续累加
      if (armed_.load()) {
        armed_ = false;
        RCLCPP_INFO(node_->get_logger(), "定位恢复 FINE, 暂停点云累积");
      }
      break;
    case 2:  // SUSPECT: 开始累积
      if (!armed_.load()) {
        armed_ = true;
        RCLCPP_WARN(node_->get_logger(), "定位进入 SUSPECT, 开始累积点云");
      }
      break;
    case 3:  // LOST: 首次立即覆盖写盘, 之后交给累计计时做增量
      if (!first_rebuild_done_ && !save_on_lost_done_) {
        save_on_lost_done_ = true;
        save_requested_ = true;
        RCLCPP_WARN(node_->get_logger(), "定位进入 LOST, 首次触发覆盖写盘");
      }
      break;
    default:
      break;
  }
}

void PriorMapManager::arm() {
  if (!armed_.load()) {
    armed_ = true;
    RCLCPP_WARN(node_->get_logger(), "收到 LOST 信号, 开始累积点云");
  }
}

void PriorMapManager::update(const PointCloudXYZI& world_cloud) {
  // 先结算可疑时长: FINE 帧也要执行, 用于结束上一段可疑状态
  const auto now = node_->get_clock()->now();
  const uint8_t state = static_cast<uint8_t>(gicp_state_.load());
  const bool suspect_now = (state == 2 || state == 3);

  if (suspect_now && !suspect_tracking_) {
    suspect_tracking_ = true;
    suspect_mark_ = now;
  } else if (!suspect_now && suspect_tracking_) {
    suspect_accum_sec_ += (now - suspect_mark_).seconds();
    suspect_tracking_ = false;
  }
  if (!suspect_now) {
    updated_this_suspect_ = false;
  }

  if (!active()) {
    return;
  }

  if (out_->z_filter.enabled && projector_ != nullptr && projector_->ready()) {
    for (const auto& point : world_cloud.points) {
      const Eigen::Vector3d world(point.x, point.y, point.z);
      const double z = projector_->apply(world).z();
      if (z >= out_->z_filter.min_z && z <= out_->z_filter.max_z) {
        buffer_->push_back(point);
      }
    }
  } else {
    buffer_->insert(buffer_->end(), world_cloud.begin(), world_cloud.end());
  }
  ++scan_count_;
  if (!buffer_->empty() && params_->pcd_save_interval > 0
      && scan_count_ >= params_->pcd_save_interval) {
    ++index_;
    const std::string path = outputPath(index_);
    if (writePcd(path, *buffer_)) {
      RCLCPP_INFO(node_->get_logger(), "current scan saved to %s",
                  path.c_str());
    }
    buffer_->clear();
    scan_count_ = 0;
  }

  // LOST 由回调置位, 在主循环里写盘, 避免跨线程操作点云
  if (save_requested_.exchange(false)) {
    saveOnLost();
  }

  // 首次丢失由 save_requested_ 走覆盖写盘, 增量只用于之后的累计触发
  if (!first_rebuild_done_ || updated_this_suspect_) {
    return;
  }
  const double delay = params_->pcd_save_suspect_build_delay_sec;
  const double total = suspect_accum_sec_
                       + (suspect_tracking_ ? (now - suspect_mark_).seconds()
                                            : 0.0);
  if (delay <= 0.0 || total < delay || buffer_->empty()) {
    return;
  }
  if (mergeMapAndWrite()) {
    RCLCPP_INFO(node_->get_logger(), "可疑状态累计 %.1f s, 已更新地图 (%zu 点)",
                total, buffer_->size());
    updated_this_suspect_ = true;
    suspect_accum_sec_ = 0.0;
    suspect_mark_ = now;
    requestReload();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "更新地图失败, 保留旧文件");
  }
}

void PriorMapManager::savePending() {
  if (buffer_->empty() || !active()) {
    return;
  }
  const std::string path = outputPath(0);
  if (writePcd(path, *buffer_)) {
    RCLCPP_INFO(node_->get_logger(), "PCD 已保存到 %s", path.c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "保存 PCD 失败: %s", path.c_str());
  }
  buffer_->clear();
  scan_count_ = 0;
}

std::string PriorMapManager::outputPath(int index) const {
  const std::string& configured = params_->pcd_save_path;
  if (!configured.empty()) {
    if (index <= 0) {
      return configured;
    }
    const std::filesystem::path path(configured);
    return (path.parent_path()
            / (path.stem().string() + "_" + std::to_string(index)
               + path.extension().string()))
        .string();
  }

  if (index > 0) {
    return std::string(ROOT_DIR) + "PCD/scans_" + std::to_string(index)
           + ".pcd";
  }

  auto t = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&t, &tm);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
  return std::string(ROOT_DIR) + "PCD/scans_" + ss.str() + ".pcd";
}

bool PriorMapManager::writePcd(const std::string& path,
                               const PointCloudXYZI& cloud) const {
  // 先解析软链接: install 目录中的 PCD 通常是指向源码的符号链接,
  // 直接 rename 会把符号链接本身替换掉。
  std::error_code ec;
  std::filesystem::path target(path);
  const auto resolved = std::filesystem::weakly_canonical(target, ec);
  if (!ec) {
    target = resolved;
  } else {
    ec.clear();
  }

  if (target.has_parent_path()) {
    std::filesystem::create_directories(target.parent_path(), ec);
    if (ec) {
      RCLCPP_ERROR(node_->get_logger(), "无法创建 PCD 目录 %s: %s",
                   target.parent_path().c_str(), ec.message().c_str());
      return false;
    }
  }

  // 先写临时文件再原子替换, 避免中途失败留下半个 PCD, 也能覆盖只读的
  // 先验地图文件 (目录可写即可)。
  const std::string tmp_path = target.string() + ".tmp";
  pcl::PCDWriter writer;
  if (writer.writeBinary(tmp_path, cloud) != 0) {
    RCLCPP_ERROR(node_->get_logger(), "写入 PCD 临时文件失败: %s",
                 tmp_path.c_str());
    return false;
  }
  std::filesystem::rename(tmp_path, target, ec);
  if (ec) {
    RCLCPP_ERROR(node_->get_logger(), "替换 PCD 文件 %s 失败: %s",
                 target.c_str(), ec.message().c_str());
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    return false;
  }
  return true;
}

void PriorMapManager::backupPriorMap() {
  const std::string path = outputPath(0);
  std::error_code ec;
  const std::filesystem::path target(path);
  if (!std::filesystem::exists(target, ec)) {
    return;
  }
  const auto stamp = std::to_string(node_->get_clock()->now().nanoseconds());
  const auto backup = target.parent_path()
                      / (target.stem().string() + "_" + stamp + ".bak.pcd");
  std::filesystem::copy_file(
      target, backup, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    RCLCPP_WARN(node_->get_logger(), "备份先验地图失败: %s",
                ec.message().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "先验地图已备份到 %s", backup.c_str());
  }
}

bool PriorMapManager::mergeMapAndWrite() {
  // 缓冲里的点云同属本次 odom 系, 直接写快照即可, 不读旧 PCD
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.setInputCloud(buffer_);
  PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
  voxel.filter(*filtered);
  if (filtered->empty()) {
    return false;
  }
  backupPriorMap();
  return writePcd(outputPath(0), *filtered);
}

void PriorMapManager::saveOnLost() {
  if (buffer_->empty()) {
    RCLCPP_WARN(node_->get_logger(), "定位丢失但累积点云为空, 跳过 PCD 保存");
    return;
  }
  const std::string path = outputPath(0);
  backupPriorMap();
  if (writePcd(path, *buffer_)) {
    RCLCPP_INFO(node_->get_logger(), "定位丢失, 累积点云已写入 %s (%zu 点)",
                path.c_str(), buffer_->size());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "定位丢失, 写入 %s 失败", path.c_str());
  }
  first_rebuild_done_ = true;
  suspect_accum_sec_ = 0.0;
  suspect_mark_ = node_->get_clock()->now();
  // 不清空缓冲: 后续快照要包含首次写盘之前的点云
  scan_count_ = 0;
  requestReload();
}

void PriorMapManager::requestReload() {
  if (!reload_client_) {
    RCLCPP_WARN(node_->get_logger(), "未配置重载服务, 先验地图保持旧文件");
    return;
  }
  if (!reload_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "重载服务不可用, 先验地图保持旧文件");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reload_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(node_->get_logger(), "先验地图重载成功: %s",
                      response->message.c_str());
        } else {
          RCLCPP_ERROR(node_->get_logger(), "先验地图重载失败: %s",
                       response->message.c_str());
        }
      });
}
