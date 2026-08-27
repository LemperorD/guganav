/**
 * @file preprocess.cpp
 * @brief 多型号 LiDAR 点云预处理实现
 *
 * 核心功能:
 * - **avia_handler**: Livox 非重复扫描处理 (点过滤 + 重复点剔除)
 * - **velodyne_handler**: Velodyne 时间戳计算 (匀速旋转模型)
 * - **oust64_handler**: Ouster 时间戳解析 (纳秒→毫秒)
 * - **hesai_handler**: 禾赛时间戳计算 (含丢失时间戳的估算)
 * - **give_feature**: 机械式雷达点云特征分类 (平面/边缘/线状)
 * - **plane_judge**: 平面判定 (基于距离方差+投影几何)
 * - **edge_jump_judge**: 跳变边缘真伪判定
 * - **process_cut_frame_***: 切帧处理 (将单帧按时间均匀切分为多子帧)
 */

#include "point_lio/preprocess.h"

/// @brief 特征提取返回值: 0=非平面, 0x10=平面+跳变?
#define RETURN0 0x00
#define RETURN0AND1 0x10

/**
 * @brief 按点时间偏移升序排序 (切帧模式下时间偏移单调递增)
 */
bool time_list_cut_frame(const PointType& x, const PointType& y) {
  return point_time_offset_ms(x) < point_time_offset_ms(y);
}

void Preprocess::configure(const PreprocessParams& params) {
  lidar_type_ = params.lidar_type;
  point_filter_num_ = params.point_filter_num;
  n_scans_ = params.scan_lines;
  scan_rate_ = params.scan_rate;
  time_unit_ = params.timestamp_unit;
  blind_ = params.blind;
  det_range_ = params.det_range;
}

// ==================== 公共处理接口 ====================

void Preprocess::process(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
    PointCloudXYZI::Ptr& pcl_out) {
  aviaHandler(msg);     // 调用 Livox 专用处理器
  *pcl_out = pl_surf_;  // 输出曲面点 (当前未提取角点)
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                         PointCloudXYZI::Ptr& pcl_out) {
  // 根据 time_unit 设置时间缩放因子
  // TODO : 太前卫了
  switch (time_unit_) {
    case SEC:
      time_unit_scale_ = 1.e3F;  // 秒 → 毫秒
      break;
    case MS:
      time_unit_scale_ = 1.F;  // 毫秒 → 毫秒
      break;
    case US:
      time_unit_scale_ = 1.e-3F;  // 微秒 → 毫秒
      break;
    case NS:
      time_unit_scale_ = 1.e-6F;  // 纳秒 → 毫秒
      break;
    default:
      time_unit_scale_ = 1.F;
      break;
  }

  // 根据雷达类型分发到对应的 handler
  switch (lidar_type_) {
    case OUST64:
      oust64Handler(msg);
      break;

    case VELO16:
      velodyneHandler(msg);
      break;

    case HESA_IXT32:
      hesaiHandler(msg);
      break;

    default:
      std::cout << "Error LiDAR Type" << '\n';
      break;
  }
  *pcl_out = pl_surf_;
}

void Preprocess::processCutFrameLivox(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
    deque<PointCloudXYZI::Ptr>& pcl_out, deque<double>& time_lidar,
    const int required_frame_num, int scan_count) {
  int plsize = (int)msg->point_num;
  pl_surf_.clear();
  pl_surf_.reserve(plsize);
  pl_full_.clear();
  pl_full_.resize(plsize);
  int valid_point_num = 0;

  for (uint i = 1; i < (uint)plsize; i++) {
    if ((msg->points[i].line < n_scans_)
        && ((msg->points[i].tag & 0x30) == 0x10
            || (msg->points[i].tag & 0x30) == 0x00)) {
      valid_point_num++;
      if (valid_point_num % point_filter_num_ == 0) {
        pl_full_[i].x = msg->points[i].x;                     // NOLINT
        pl_full_[i].y = msg->points[i].y;                     // NOLINT
        pl_full_[i].z = msg->points[i].z;                     // NOLINT
        pl_full_[i].intensity = msg->points[i].reflectivity;  // NOLINT
        // PCL curvature 字段承载每个点的时间偏移，单位为 ms。
        pl_full_[i].curvature = msg->points[i].offset_time  // NOLINT
                                / float(1000000);

        double dist = pl_full_[i].getVector3fMap().squaredNorm();
        if (dist < blind_ * blind_ || dist > det_range_ * det_range_) {
          continue;
        }

        if ((abs(pl_full_[i].x - pl_full_[i - 1].x) > 1e-7)
            || (abs(pl_full_[i].y - pl_full_[i - 1].y) > 1e-7)
            || (abs(pl_full_[i].z - pl_full_[i - 1].z) > 1e-7)) {
          pl_surf_.push_back(pl_full_[i]);
        }
      }
    }
  }
  sort(pl_surf_.points.begin(), pl_surf_.points.end(), time_list_cut_frame);
  // end time of last frame，单位ms
  double last_frame_end_time = rclcpp::Time(msg->header.stamp).seconds() * 1000;
  uint valid_num = 0;
  uint cut_num = 0;
  uint valid_pcl_size = pl_surf_.points.size();

  int required_cut_num = required_frame_num;
  if (scan_count < 5) {
    required_cut_num = 1;
  }

  PointCloudXYZI pcl_cut;
  for (uint i = 1; i < valid_pcl_size; i++) {
    valid_num++;
    // Compute new opffset time of each point：ms
    pl_surf_[i].curvature += (float)(rclcpp::Time(msg->header.stamp).seconds()
                                     * 1000)
                             - last_frame_end_time;
    pcl_cut.push_back(pl_surf_[i]);
    if (valid_num
        == (uint)(int((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
      cut_num++;
      time_lidar.push_back(last_frame_end_time);
      PointCloudXYZI::Ptr pcl_temp =
          std::make_shared<PointCloudXYZI>();  // Initialize shared_ptr
      *pcl_temp = pcl_cut;
      pcl_out.push_back(pcl_temp);
      // Update frame head
      last_frame_end_time += pl_surf_[i].curvature;
      pcl_cut.clear();
      pcl_cut.reserve(valid_pcl_size * 2 / required_frame_num);
    }
  }
}
void Preprocess::processCutFramePCL2(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    deque<PointCloudXYZI::Ptr>& pcl_out, deque<double>& time_lidar,
    const int required_frame_num, int scan_count) {
  pl_surf_.clear();
  pl_full_.clear();
  if (lidar_type_ == VELO16) {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    auto plsize = pl_orig.points.size();
    pl_surf_.reserve(plsize);

    bool is_first[MAX_LINE_NUM];
    double yaw_fp[MAX_LINE_NUM] = {0};      // yaw of first scan point
    double omega_l = 3.61;                  // scan angular velocity (deg/ms)
    float time_last[MAX_LINE_NUM] = {0.0};  // last offset time

    if (pl_orig.points[plsize - 1].time > 0) {
      given_offset_time_ = true;
    } else {
      std::cout << "Compute offset time using constant rotation model." << '\n';
      given_offset_time_ = false;
      memset(is_first, true, sizeof(is_first));
    }

    for (std::size_t i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * 1000.0;  // ms

      double dist = added_pt.getVector3fMap().squaredNorm();
      if (dist < blind_ * blind_ || dist > det_range_ * det_range_
          || isnan(added_pt.x) || isnan(added_pt.y)  // NOLINT
          || isnan(added_pt.z)) {                    // NOLINT
        continue;
      }

      if (!given_offset_time_) {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer]) {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          time_last[layer] = added_pt.curvature;
          continue;
        }
        // compute offset time
        if (yaw_angle <= yaw_fp[layer]) {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }
        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num_ == 0 && pl_orig.points[i].ring < n_scans_) {
        pl_surf_.points.push_back(added_pt);
      }
    }
  } else if (lidar_type_ == OUST64) {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf_.reserve(plsize);
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].t / 1e6;  // ns to ms

      double dist = added_pt.getVector3fMap().squaredNorm();
      if (dist < blind_ * blind_ || dist > det_range_ * det_range_
          || isnan(added_pt.x) || isnan(added_pt.y) || isnan(added_pt.z)) {
        continue;
      }
      if (i % point_filter_num_ == 0 && pl_orig.points[i].ring < n_scans_) {
        pl_surf_.points.push_back(added_pt);
      }
    }
  } else if (lidar_type_ == HESA_IXT32) {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf_.reserve(plsize);
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp
                            - rclcpp::Time(msg->header.stamp).seconds())
                           * 1000;  // s to ms

      double dist = added_pt.getVector3fMap().squaredNorm();
      if (dist < blind_ * blind_ || dist > det_range_ * det_range_
          || isnan(added_pt.x) || isnan(added_pt.y) || isnan(added_pt.z)) {
        continue;
      }

      if (i % point_filter_num_ == 0 && pl_orig.points[i].ring < n_scans_) {
        pl_surf_.points.push_back(added_pt);
      }
    }
  } else {
    std::cout << "Wrong LiDAR Type!!!" << '\n';
    return;
  }

  sort(pl_surf_.points.begin(), pl_surf_.points.end(), time_list_cut_frame);

  // ms
  double last_frame_end_time = rclcpp::Time(msg->header.stamp).seconds() * 1000;
  uint valid_num = 0;
  uint cut_num = 0;
  uint valid_pcl_size = pl_surf_.points.size();

  int required_cut_num = required_frame_num;

  if (scan_count < 20) {
    required_cut_num = 1;
  }

  PointCloudXYZI pcl_cut;
  for (uint i = 1; i < valid_pcl_size; i++) {
    valid_num++;
    pl_surf_[i].curvature += rclcpp::Time(msg->header.stamp).seconds() * 1000
                             - last_frame_end_time;
    pcl_cut.push_back(pl_surf_[i]);

    if (valid_num
        == (uint((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
      cut_num++;
      time_lidar.push_back(last_frame_end_time);
      PointCloudXYZI::Ptr pcl_temp(new PointCloudXYZI());
      *pcl_temp = pcl_cut;
      pcl_out.push_back(pcl_temp);
      last_frame_end_time += pl_surf_[i].curvature;
      pcl_cut.clear();
      pcl_cut.reserve(valid_pcl_size * 2 / required_frame_num);
    }
  }
}

/**
 * @brief Livox 非重复扫描点云处理 (avia_handler)
 *
 * 处理 Livox CustomMsg 格式:
 * - 点过滤: 只保留线号 < N_SCANS 且 tag = 0x10/0x00 的点
 * - 降采样: 每隔 point_filter_num 取1点
 * - 盲区/远距过滤: blind² < dist² < det_range²
 * - 重复点剔除: 坐标与上一点完全相同则跳过 (Livox 非重复扫描可能有重复)
 * - 时间戳: offset_time (ns) → 点时间偏移 (ms)
 */
void Preprocess::aviaHandler(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  pl_surf_.clear();
  pl_full_.clear();
  int plsize = msg->point_num;  // 该帧总点数

  pl_surf_.reserve(plsize);
  pl_full_.resize(plsize);
  uint valid_num = 0;

  for (int i = 1; i < plsize; i++) {
    // ---- 线号 + tag 过滤 ----
    // Livox tag: bit4-5 标识回波类型 (0x10=最强回波, 0x00=首次回波,
    // 0x20=最末回波)
    if ((msg->points[i].line < n_scans_)
        && ((msg->points[i].tag & 0x30) == 0x10
            || (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (valid_num % point_filter_num_ == 0) {
        pl_full_[i].x = msg->points[i].x;
        pl_full_[i].y = msg->points[i].y;
        pl_full_[i].z = msg->points[i].z;
        pl_full_[i].intensity = msg->points[i].reflectivity;

        // offset_time 单位为纳秒 → 转换为毫秒存入 PCL 时间字段。
        pl_full_[i].curvature = msg->points[i].offset_time
                                / float(1000000);  // ns → ms

        // ---- 距离过滤 (盲区 + 最大距离) ----
        double dist = pl_full_[i].getVector3fMap().squaredNorm();
        if (dist < blind_ * blind_ || dist > det_range_ * det_range_)
          continue;

        // ---- 重复点剔除: 坐标与上一点完全相同则跳过 ----
        if (((abs(pl_full_[i].x - pl_full_[i - 1].x) > 1e-7)
             || (abs(pl_full_[i].y - pl_full_[i - 1].y) > 1e-7)
             || (abs(pl_full_[i].z - pl_full_[i - 1].z) > 1e-7))) {
          pl_surf_.push_back(pl_full_[i]);
        }
      }
    }
  }
}

/**
 * @brief Ouster OS1-64 点云处理
 *
 * 解析 Ouster 的 UInt32 纳秒时间戳，转换为毫秒点时间偏移。
 * 点过滤: 降采样 + 盲区/远距/NaN 过滤
 */
void Preprocess::oust64Handler(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  pl_surf_.clear();
  pl_full_.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf_.reserve(plsize);

  for (size_t i = 0; i < pl_orig.points.size(); i++) {
    if (i % point_filter_num_ != 0) {
      continue;  // 降采样
    }
    double range = pl_orig.points[i].x * pl_orig.points[i].x
                   + pl_orig.points[i].y * pl_orig.points[i].y
                   + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < (blind_ * blind_) || range > det_range_ * det_range_
        || isnan(pl_orig.points[i].x) || isnan(pl_orig.points[i].y)
        || isnan(pl_orig.points[i].z))
      continue;  // 盲区/远距/NaN 过滤

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    // Ouster 的 t 字段为纳秒偏移 → ms (time_unit_scale 控制)
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale_;

    pl_surf_.points.push_back(added_pt);
  }
}

/**
 * @brief Velodyne VLP-16 点云处理
 *
 * 关键: 如果雷达未提供时间戳 (given_offset_time = false),
 * 用匀速旋转模型估算每个点的偏移时间:
 *   offset_time = azimuth_delta / omega_l
 *
 * omega_l = 旋转角速度 (度/ms) = 360° * SCAN_RATE / 1000 ≈ 3.61°/ms @ 10Hz
 */
void Preprocess::velodyneHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  pl_surf_.clear();
  pl_full_.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const auto plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf_.reserve(plsize);

  /*** 当雷达不提供时间戳时使用的变量 ***/
  double omega_l = 0.361 * scan_rate_;  // 扫描角速度 (度/ms) = 360*Hz/1000
  std::vector<bool> is_first(n_scans_, true);
  std::vector<double> yaw_fp(n_scans_, 0.0);    // 每条线的首点方位角
  std::vector<float> time_last(n_scans_, 0.0);  // 每条线的上一点偏移时间
  /*****************************************************************/

  // 检查是否有点时间戳 (最后一个点的时间 > 0)
  if (pl_orig.points[plsize - 1].time > 0) {
    given_offset_time_ = true;  // 使用硬件时间戳
  } else {
    given_offset_time_ = false;  // 用匀速旋转模型估算
  }

  for (std::size_t i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = pl_orig.points[i].time
                         * time_unit_scale_;  // 点时间偏移，单位 ms

    if (i % point_filter_num_ != 0 || std::isnan(added_pt.x)
        || std::isnan(added_pt.y) || std::isnan(added_pt.z))
      continue;

    if (!given_offset_time_) {
      // ---- 用均匀角速度模型估算时间偏移 ----
      int layer = 0;  // 注意: VLP-16 的 ring 域可能为空，硬编码 layer=0
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // rad→度

      if (is_first[layer]) {
        // 每个 layer 的第一个点: 记录参考 yaw
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;  // 首点时间 = 0
        time_last[layer] = added_pt.curvature;
        continue;
      }

      // 计算角度差 → 时间偏移
      // 如果 yaw < yaw_fp (跨 0°), 补 360° 避免负值
      if (yaw_angle < yaw_fp[layer]) {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      } else {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      // ---- 下面的跨周期处理被注释掉了 ----
      // if (added_pt.curvature < time_last[layer])
      // added_pt.curvature+=360.0/omega_l;
    }

    // 距离过滤
    double dist = added_pt.getVector3fMap().squaredNorm();
    {
      if (dist > (blind_ * blind_) && dist < (det_range_ * det_range_)) {
        pl_surf_.points.push_back(added_pt);
      }
    }
  }
}

/**
 * @brief 禾赛 XT32 点云处理
 *
 * 与 Velodyne 类似，使用匀速旋转模型估算时间戳。
 * 区别: 禾赛 timestamp 字段为绝对时间 (秒)，需转换为相对偏移时间。
 * 点时间偏移 = (timestamp - time_head) * time_unit_scale (ms)
 */
void Preprocess::hesaiHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  pl_surf_.clear();
  pl_full_.clear();

  pcl::PointCloud<hesai_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  auto plsize = pl_orig.points.size();
  if (plsize == 0) {
    return;
  }
  pl_surf_.reserve(plsize);

  /*** 当雷达不提供时间戳时使用的变量 ***/
  double omega_l = 0.361 * scan_rate_;  // 扫描角速度 (度/ms)
  std::vector<bool> is_first(n_scans_, true);
  std::vector<double> yaw_fp(n_scans_, 0.0);    // 每条线的首点方位角
  std::vector<float> time_last(n_scans_, 0.0);  // 每条线的上一点偏移时间
  /*****************************************************************/

  if (pl_orig.points[plsize - 1].timestamp > 0) {
    given_offset_time_ = true;  // 有硬件时间戳
  } else {
    given_offset_time_ = false;  // 用模型估算
  }

  double time_head = pl_orig.points[0].timestamp;  // 帧首点绝对时间

  for (std::size_t i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;

    // 相对偏移时间: (当前点绝对时间 - 首点绝对时间) * 缩放 → ms
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head)
                         * time_unit_scale_;

    if (!given_offset_time_) {
      // ---- 匀速旋转模型估计偏移时间 ----
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;  // 首点偏移 0
        time_last[layer] = added_pt.curvature;
        continue;
      }

      // 计算角度差 → 时间偏移 (处理跨 360° 情况)
      if (yaw_angle <= yaw_fp[layer]) {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      } else {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      // 跨周期处理: 如果计算的时间小于上一点的时间，补一个周期
      if (added_pt.curvature < time_last[layer])
        added_pt.curvature += 360.0 / omega_l;

      time_last[layer] = added_pt.curvature;
    }

    // 降采样 + NaN过滤 + 距离过滤
    if (i % point_filter_num_ == 0 && !std::isnan(added_pt.x)
        && !std::isnan(added_pt.y) && !std::isnan(added_pt.z)) {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y
                  + added_pt.z * added_pt.z
              > (blind_ * blind_)
          && added_pt.x * added_pt.x + added_pt.y * added_pt.y
                     + added_pt.z * added_pt.z
                 < (det_range_ * det_range_)) {
        pl_surf_.points.push_back(added_pt);
      }
    }
  }
}
