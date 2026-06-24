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

#include "preprocess.h"

/// @brief 特征提取返回值: 0=非平面, 0x10=平面+跳变?
#define RETURN0 0x00
#define RETURN0AND1 0x10

/**
 * @brief 按 curvature 升序排序 (切帧模式下时间戳单调递增)
 */
const bool time_list_cut_frame(PointType & x, PointType & y) { return (x.curvature < y.curvature); }

/**
 * @brief 构造函数: 初始化经验参数
 *
 * 参数意义:
 * - inf_bound: 无穷远边界 (用于跳变判断)
 * - group_size: 点组大小 (连续 group_size 个点参与平面判定)
 * - disA/disB: 距离容限 = disA*range + disB
 * - p2l_ratio: 平面投影宽度/长度比上限
 * - jump_up_limit: 远跳变判定角度 cos(170°)
 * - jump_down_limit: 近跳变判定角度 cos(8°)
 * - cos160: 边缘夹角判定 cos(160°)
 * - edgea/edgeb: 跳变判定参数 (距离比+绝对差)
 * - smallp_intersect/ratio: 小平面判定阈值
 */
Preprocess::Preprocess() : lidar_type(AVIA), blind(0.01), point_filter_num(1), det_range(1000)
{
  inf_bound = 10;
  N_SCANS = 6;        // Livox 默认 6 线 (实际由 YAML 覆盖)
  SCAN_RATE = 10;     // 默认 10Hz
  group_size = 8;     // 每组 8 个点
  disA = 0.01;
  disA = 0.1;   // 注意: 覆盖了上面的 disA, 实际值 0.1
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;     // 度 (后续转 cos)
  jump_down_limit = 8.0;     // 度 (后续转 cos)
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  // 度 → cos 值转换
  jump_up_limit = cos(jump_up_limit / 180 * M_PI);      // cos(170°)
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);  // cos(8°)
  cos160 = cos(cos160 / 180 * M_PI);                    // cos(160°)
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);// cos(172.5°)
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

// ==================== 公共处理接口 ====================

void Preprocess::process(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg, PointCloudXYZI::Ptr & pcl_out)
{
  avia_handler(msg);      // 调用 Livox 专用处理器
  *pcl_out = pl_surf;      // 输出曲面点 (当前未提取角点)
}

void Preprocess::process(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg, PointCloudXYZI::Ptr & pcl_out)
{
  // 根据 time_unit 设置时间缩放因子
  switch (time_unit) {
    case SEC:
      time_unit_scale = 1.e3f;   // 秒 → 毫秒
      break;
    case MS:
      time_unit_scale = 1.f;     // 毫秒 → 毫秒
      break;
    case US:
      time_unit_scale = 1.e-3f;  // 微秒 → 毫秒
      break;
    case NS:
      time_unit_scale = 1.e-6f;  // 纳秒 → 毫秒
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  // 根据雷达类型分发到对应的 handler
  switch (lidar_type) {
    case OUST64:
      oust64_handler(msg);
      break;

    case VELO16:
      velodyne_handler(msg);
      break;

    case HESAIxt32:
      hesai_handler(msg);
      break;

    default:
      printf("Error LiDAR Type");
      break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::process_cut_frame_livox(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg, deque<PointCloudXYZI::Ptr> & pcl_out,
  deque<double> & time_lidar, const int required_frame_num, int scan_count)
{
  int plsize = msg->point_num;
  pl_surf.clear();
  pl_surf.reserve(plsize);
  pl_full.clear();
  pl_full.resize(plsize);
  int valid_point_num = 0;

  for (uint i = 1; i < plsize; i++) {
    if (
      (msg->points[i].line < N_SCANS) &&
      ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
      valid_point_num++;
      if (valid_point_num % point_filter_num == 0) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        //use curvature as time of each laser points，unit: ms
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

        double dist =
          pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z;
        if (dist < blind * blind || dist > det_range * det_range) continue;

        if (
          (abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
          (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
          (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
          pl_surf.push_back(pl_full[i]);
        }
      }
    }
  }
  sort(pl_surf.points.begin(), pl_surf.points.end(), time_list_cut_frame);
  //end time of last frame，单位ms
  double last_frame_end_time = rclcpp::Time(msg->header.stamp).seconds() * 1000;
  uint valid_num = 0;
  uint cut_num = 0;
  uint valid_pcl_size = pl_surf.points.size();

  int required_cut_num = required_frame_num;
  if (scan_count < 5) required_cut_num = 1;

  PointCloudXYZI pcl_cut;
  for (uint i = 1; i < valid_pcl_size; i++) {
    valid_num++;
    //Compute new opffset time of each point：ms
    pl_surf[i].curvature += rclcpp::Time(msg->header.stamp).seconds() * 1000 - last_frame_end_time;
    pcl_cut.push_back(pl_surf[i]);
    if (valid_num == (int((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
      cut_num++;
      time_lidar.push_back(last_frame_end_time);
      PointCloudXYZI::Ptr pcl_temp(new PointCloudXYZI());  //Initialize shared_ptr
      *pcl_temp = pcl_cut;
      pcl_out.push_back(pcl_temp);
      //Update frame head
      last_frame_end_time += pl_surf[i].curvature;
      pcl_cut.clear();
      pcl_cut.reserve(valid_pcl_size * 2 / required_frame_num);
    }
  }
}
#define MAX_LINE_NUM 128
void Preprocess::process_cut_frame_pcl2(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg, deque<PointCloudXYZI::Ptr> & pcl_out,
  deque<double> & time_lidar, const int required_frame_num, int scan_count)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  if (lidar_type == VELO16) {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);

    bool is_first[MAX_LINE_NUM];
    double yaw_fp[MAX_LINE_NUM] = {0};      // yaw of first scan point
    double omega_l = 3.61;                  // scan angular velocity (deg/ms)
    float yaw_last[MAX_LINE_NUM] = {0.0};   // yaw of last scan point
    float time_last[MAX_LINE_NUM] = {0.0};  // last offset time

    if (pl_orig.points[plsize - 1].time > 0) {
      given_offset_time = true;
    } else {
      std::cout << "Compute offset time using constant rotation model." << '\n';
      given_offset_time = false;
      memset(is_first, true, sizeof(is_first));
    }

    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * 1000.0;  //ms

      double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
      if (
        dist < blind * blind || dist > det_range * det_range || isnan(added_pt.x) ||
        isnan(added_pt.y) || isnan(added_pt.z))
        continue;

      if (!given_offset_time) {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer]) {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }
        // compute offset time
        if (yaw_angle <= yaw_fp[layer]) {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }
        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0 && pl_orig.points[i].ring < N_SCANS) {
        pl_surf.points.push_back(added_pt);
      }
    }
  } else if (lidar_type == OUST64) {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].t / 1e6;  //ns to ms

      double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
      if (
        dist < blind * blind || dist > det_range * det_range || isnan(added_pt.x) ||
        isnan(added_pt.y) || isnan(added_pt.z))
        continue;

      if (i % point_filter_num == 0 && pl_orig.points[i].ring < N_SCANS) {
        pl_surf.points.push_back(added_pt);
      }
    }
  } else if (lidar_type == HESAIxt32) {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);
    for (int i = 0; i < plsize; i++) {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature =
        (pl_orig.points[i].timestamp - rclcpp::Time(msg->header.stamp).seconds()) * 1000;  //s to ms

      double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
      if (
        dist < blind * blind || dist > det_range * det_range || isnan(added_pt.x) ||
        isnan(added_pt.y) || isnan(added_pt.z))
        continue;

      if (i % point_filter_num == 0 && pl_orig.points[i].ring < N_SCANS) {
        pl_surf.points.push_back(added_pt);
      }
    }
  } else {
    std::cout << "Wrong LiDAR Type!!!" << '\n';
    return;
  }

  sort(pl_surf.points.begin(), pl_surf.points.end(), time_list_cut_frame);

  //ms
  double last_frame_end_time = rclcpp::Time(msg->header.stamp).seconds() * 1000;
  uint valid_num = 0;
  uint cut_num = 0;
  uint valid_pcl_size = pl_surf.points.size();

  int required_cut_num = required_frame_num;

  if (scan_count < 20) required_cut_num = 1;

  PointCloudXYZI pcl_cut;
  for (uint i = 1; i < valid_pcl_size; i++) {
    valid_num++;
    pl_surf[i].curvature += rclcpp::Time(msg->header.stamp).seconds() * 1000 - last_frame_end_time;
    pcl_cut.push_back(pl_surf[i]);

    if (valid_num == (int((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
      cut_num++;
      time_lidar.push_back(last_frame_end_time);
      PointCloudXYZI::Ptr pcl_temp(new PointCloudXYZI());
      *pcl_temp = pcl_cut;
      pcl_out.push_back(pcl_temp);
      last_frame_end_time += pl_surf[i].curvature;
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
 * - 时间戳: offset_time (ns) → curvature (ms)
 */
void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;  // 该帧总点数

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 按线号清空缓冲区
  for (int i = 0; i < N_SCANS; i++) {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  for (uint i = 1; i < plsize; i++) {
    // ---- 线号 + tag 过滤 ----
    // Livox tag: bit4-5 标识回波类型 (0x10=最强回波, 0x00=首次回波, 0x20=最末回波)
    if (
      (msg->points[i].line < N_SCANS) &&
      ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (valid_num % point_filter_num == 0) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;

        // offset_time 单位为纳秒 → 转换为毫秒存入 curvature
        // (curvature 域复用为时间戳存储)
        pl_full[i].curvature =
          msg->points[i].offset_time / float(1000000);  // ns → ms

        // ---- 距离过滤 (盲区 + 最大距离) ----
        double dist =
          pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z;
        if (dist < blind * blind || dist > det_range * det_range) continue;

        // ---- 重复点剔除: 坐标与上一点完全相同则跳过 ----
        if (((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
             (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
             (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))) {
          pl_surf.push_back(pl_full[i]);
        }
      }
    }
  }
}

/**
 * @brief Ouster OS1-64 点云处理
 *
 * 解析 Ouster 的 UInt32 纳秒时间戳，转换为毫秒存入 curvature。
 * 点过滤: 降采样 + 盲区/远距/NaN 过滤
 */
void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = rclcpp::Time(msg->header.stamp).seconds();
  for (int i = 0; i < pl_orig.points.size(); i++) {
    if (i % point_filter_num != 0) continue;  // 降采样

    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;

    if (
      range < (blind * blind) || range > det_range * det_range || isnan(pl_orig.points[i].x) ||
      isnan(pl_orig.points[i].y) || isnan(pl_orig.points[i].z))
      continue;  // 盲区/远距/NaN 过滤

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    // Ouster 的 t 字段为纳秒偏移 → ms (time_unit_scale 控制)
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale;

    pl_surf.points.push_back(added_pt);
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
void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  /*** 当雷达不提供时间戳时使用的变量 ***/
  double omega_l = 0.361 * SCAN_RATE;    // 扫描角速度 (度/ms) = 360*Hz/1000
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);    // 每条线的首点方位角
  std::vector<float> yaw_last(N_SCANS, 0.0);   // 每条线的末点方位角
  std::vector<float> time_last(N_SCANS, 0.0);  // 每条线的上一点偏移时间
  /*****************************************************************/

  // 检查是否有点时间戳 (最后一个点的时间 > 0)
  if (pl_orig.points[plsize - 1].time > 0) {
    given_offset_time = true;   // 使用硬件时间戳
  } else {
    given_offset_time = false;  // 用匀速旋转模型估算
  }

  for (int i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms

    if (
      i % point_filter_num != 0 || std::isnan(added_pt.x) || std::isnan(added_pt.y) ||
      std::isnan(added_pt.z))
      continue;

    if (!given_offset_time) {
      // ---- 用均匀角速度模型估算时间偏移 ----
      int layer = 0;  // 注意: VLP-16 的 ring 域可能为空，硬编码 layer=0
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // rad→度

      if (is_first[layer]) {
        // 每个 layer 的第一个点: 记录参考 yaw
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;  // 首点时间 = 0
        yaw_last[layer] = yaw_angle;
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
      // if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;
    }

    // 距离过滤
    double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
    {
      if (dist > (blind * blind) && dist < (det_range * det_range)) {
        pl_surf.points.push_back(added_pt);
      }
    }
  }
}

/**
 * @brief 禾赛 XT32 点云处理
 *
 * 与 Velodyne 类似，使用匀速旋转模型估算时间戳。
 * 区别: 禾赛 timestamp 字段为绝对时间 (秒)，需转换为相对偏移时间。
 * curvature = (timestamp - time_head) * time_unit_scale (ms)
 */
void Preprocess::hesai_handler(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<hesai_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  /*** 当雷达不提供时间戳时使用的变量 ***/
  double omega_l = 0.361 * SCAN_RATE;    // 扫描角速度 (度/ms)
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);    // 每条线的首点方位角
  std::vector<float> yaw_last(N_SCANS, 0.0);   // 每条线的末点方位角
  std::vector<float> time_last(N_SCANS, 0.0);  // 每条线的上一点偏移时间
  /*****************************************************************/

  if (pl_orig.points[plsize - 1].timestamp > 0) {
    given_offset_time = true;   // 有硬件时间戳
  } else {
    given_offset_time = false;  // 用模型估算
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--) {
      if (pl_orig.points[i].ring == layer_first) {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  double time_head = pl_orig.points[0].timestamp;  // 帧首点绝对时间

  for (int i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;

    // 相对偏移时间: (当前点绝对时间 - 首点绝对时间) * 缩放 → ms
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * time_unit_scale;

    if (!given_offset_time) {
      // ---- 匀速旋转模型估计偏移时间 ----
      int layer = pl_orig.points[i].ring;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;  // 首点偏移 0
        yaw_last[layer] = yaw_angle;
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
      if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
    }

    // 降采样 + NaN过滤 + 距离过滤
    if (
      i % point_filter_num == 0 && !std::isnan(added_pt.x) && !std::isnan(added_pt.y) &&
      !std::isnan(added_pt.z)) {
      if (
        added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
          (blind * blind) &&
        added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z <
          (det_range * det_range)) {
        pl_surf.points.push_back(added_pt);
      }
    }
  }
}

// ==================== 特征提取 (机械旋转式雷达专用) ====================

/**
 * @brief 特征分类主函数: 对每条扫描线独立进行点特征分类
 *
 * 处理流程:
 * 1. 跳过盲区点
 * 2. plane_judge(): 遍历扫描线，将连续点分组判断是否为平面
 *    - 如果是平面 (plane_type=1): 端点标记 Poss_Plane, 内部点标记 Real_Plane
 *    - 如果连续两个平面方向夹角 > 45°: 连接点标记为 Edge_Plane
 * 3. 跳变边缘检测 (edge_jump_judge):
 *    - Nr_180: 大角度跳变 (遮挡边界)
 *    - Nr_zero: 小角度跳变
 *    - 满足条件的跳变点标记 Edge_Jump
 * 4. 小平面精化: 角度连续 + 距离比小的短段标记为 Real_Plane
 * 5. 降采样滤波: 平面点每隔 point_filter_num 取1点输出
 *
 * @param pl 输入点云 (按扫描线排列)
 * @param types 输入/输出特征类型数组
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> & pl, vector<orgtype> & types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0) {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while (types[head].range < blind) {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++) {
    if (types[i].range < blind) {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1) {
      for (uint j = i; j <= i_nex; j++) {
        if (j != i && j != i_nex) {
          types[j].ftype = Real_Plane;
        } else {
          types[j].ftype = Poss_Plane;
        }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if (last_state == 1 && last_direct.norm() > 0.1) {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707) {
          types[i].ftype = Edge_Plane;
        } else {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    } else  // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++) {
    if (types[i].range < blind || types[i].ftype >= Real_Plane) {
      continue;
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++) {
      int m = -1;
      if (j == 1) {
        m = 1;
      }

      if (types[i + m].range < blind) {
        if (types[i].range > inf_bound) {
          types[i].edj[j] = Nr_inf;
        } else {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit) {
        types[i].edj[j] = Nr_180;
      } else if (types[i].angle[j] > jump_down_limit) {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if (
      types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
      types[i].dista > 4 * types[i - 1].dista) {
      if (types[i].intersect > cos160) {
        if (edge_jump_judge(pl, types, i, Prev)) {
          types[i].ftype = Edge_Jump;
        }
      }
    } else if (
      types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor &&
      types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista) {
      if (types[i].intersect > cos160) {
        if (edge_jump_judge(pl, types, i, Next)) {
          types[i].ftype = Edge_Jump;
        }
      }
    } else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
      if (edge_jump_judge(pl, types, i, Prev)) {
        types[i].ftype = Edge_Jump;
      }
    } else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
      if (edge_jump_judge(pl, types, i, Next)) {
        types[i].ftype = Edge_Jump;
      }

    } else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
      if (types[i].ftype == Nor) {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++) {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
      continue;
    }

    if (types[i].ftype == Nor) {
      if (types[i - 1].dista > types[i].dista) {
        ratio = types[i - 1].dista / types[i].dista;
      } else {
        ratio = types[i].dista / types[i - 1].dista;
      }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
        if (types[i - 1].ftype == Nor) {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor) {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < plsize; j++) {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
      if (last_surface == -1) {
        last_surface = j;
      }

      if (j == uint(last_surface + point_filter_num - 1)) {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    } else {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
        pl_corn.push_back(pl[j]);
      }
      if (last_surface != -1) {
        PointType ap;
        for (uint k = last_surface; k < j; k++) {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

/**
 * @brief 连续点组的平面判定
 *
 * 判定原理:
 * 1. 在扫描线上从 i_cur 出发，收集满足距离阈值的连续点
 * 2. 计算首末点连线方向 v = p_end - p_start
 * 3. 计算每个中间点到首末点连线的垂直距离 (叉积的模)
 * 4. 如果最大垂直距离 / 首末距离² < p2l_ratio 阈值 → 疑似平面
 * 5. AVIA: 额外检查距离分布的 max/mid 和 mid/min 比值
 * 6. 机械雷达: 检查 max/min 比值
 *
 * @param pl 输入点云
 * @param types 特征类型数组
 * @param i_cur 当前起始点索引
 * @param[out] i_nex 平面结束点索引 (下一个不属于此平面的点)
 * @param[out] curr_direct 平面方向单位向量
 * @return 1=平面有效, 0=非平面, 2=遇到盲区中断
 */
int Preprocess::plane_judge(
  const PointCloudXYZI & pl, vector<orgtype> & types, uint i_cur, uint & i_nex,
  Eigen::Vector3d & curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB;  // 距离容限
  group_dis = group_dis * group_dis;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  // Step 1: 前向扩展，收集满足距离容限的连续点
  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
    if (types[i_nex].range < blind) {
      curr_direct.setZero();
      return 2;  // 盲区中断
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;) {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if (types[i_nex].range < blind) {
      curr_direct.setZero();
      return 2;
    }
    // 当前点与起始点的欧氏距离平方
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis) {
      break;  // 超出距离容限，停止扩展
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  // Step 2: 计算每个中间点到首末点连线的最大垂直距离 (平面性指标)
  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++) {
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    // v2 = v1 × v_direction (垂直于首末点连线方向的偏离)
    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid) {
      leng_wid = lw;  // 记录最大垂直偏离平方
    }
  }

  // 距离平方 / 最大垂直偏离 > p2l_ratio → 非平面 (太弯曲)
  if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
    curr_direct.setZero();
    return 0;
  }

  // Step 3: 对 dista 数组升序排列 (选择排序)
  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++) {
    for (uint k = j + 1; k < disarrsize; k++) {
      if (disarr[j] < disarr[k]) {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16) {
    curr_direct.setZero();
    return 0;
  }

  // Step 4: 距离方差检验 (排除边缘/角点)
  if (lidar_type == AVIA) {
    // AVIA: 检查 max/mid 和 mid/min 比值
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
      curr_direct.setZero();
      return 0;
    }
  } else {
    // 机械雷达: 检查 max/min 比值
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin) {
      curr_direct.setZero();
      return 0;
    }
  }

  // 平面有效: 设置方向向量并归一化
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

/**
 * @brief 跳变边缘真伪判定
 *
 * 判断当前点前后邻域的距离突变是否为真实的几何边缘 (非遮挡/噪声):
 * - 取 nor_dir 方向的两个邻点
 * - 比较两个邻点的 dista (到前后点连线的距离)
 * - 如果大的 dista / 小的 dista < edgea 且差值 < edgeb → 真边缘
 *
 * @param nor_dir 判断方向: Prev (前向) 或 Next (后向)
 * @return true=真边缘跳变, false=假跳变
 */
bool Preprocess::edge_jump_judge(
  const PointCloudXYZI & pl, vector<orgtype> & types, uint i, Surround nor_dir)
{
  if (nor_dir == 0) {
    if (types[i - 1].range < blind || types[i - 2].range < blind) {
      return false;
    }
  } else if (nor_dir == 1) {
    if (types[i + 1].range < blind || types[i + 2].range < blind) {
      return false;
    }
  }
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  // 确保 d1 >= d2
  if (d1 < d2) {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // 距离比检验 + 绝对差检验
  if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
    return false;  // 不满足边缘判据
  }

  return true;
}
