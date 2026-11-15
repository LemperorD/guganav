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
 */

#include "point_lio/utilities/preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

void Preprocess::configure(const PreprocessParams& params) {
  lidar_type_ = params.lidar_type;
  point_filter_num_ = params.point_filter_num;
  n_scans_ = params.scan_lines;
  scan_rate_ = params.scan_rate;
  time_unit_ = params.timestamp_unit;
  blind_ = params.blind;
  det_range_ = params.det_range;
}

void Preprocess::process(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
    PointCloudXYZI::Ptr& pcl_out) {
  aviaHandler(msg);
  *pcl_out = pl_surf_;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                         PointCloudXYZI::Ptr& pcl_out) {
  // TODO: 太前卫了
  switch (time_unit_) {
    case SEC:
      time_unit_scale_ = 1.e3F;
      break;
    case MS:
      time_unit_scale_ = 1.F;
      break;
    case US:
      time_unit_scale_ = 1.e-3F;
      break;
    case NS:
      time_unit_scale_ = 1.e-6F;
      break;
    default:
      time_unit_scale_ = 1.F;
      break;
  }

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
ImuMeasurement Preprocess::processImu(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  ImuMeasurement measurement;
  measurement.angular_velocity << msg->angular_velocity.x,
      msg->angular_velocity.y, msg->angular_velocity.z;
  measurement.linear_acceleration << msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z;
  return measurement;
}

void Preprocess::aviaHandler(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
  pl_surf_.clear();
  pl_full_.clear();
  int plsize = msg->point_num;

  pl_surf_.reserve(plsize);
  pl_full_.resize(plsize);
  uint valid_num = 0;

  for (int i = 1; i < plsize; i++) {
    if ((msg->points[i].line < n_scans_)
        && ((msg->points[i].tag & 0x30) == 0x10
            || (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (valid_num % point_filter_num_ == 0) {
        pl_full_[i].x = msg->points[i].x;
        pl_full_[i].y = msg->points[i].y;
        pl_full_[i].z = msg->points[i].z;
        pl_full_[i].intensity = msg->points[i].reflectivity;

        pl_full_[i].curvature = msg->points[i].offset_time / float(1000000);

        double dist = pl_full_[i].getVector3fMap().squaredNorm();
        if (dist < blind_ * blind_ || dist > det_range_ * det_range_)
          continue;

        if (((abs(pl_full_[i].x - pl_full_[i - 1].x) > 1e-7)
             || (abs(pl_full_[i].y - pl_full_[i - 1].y) > 1e-7)
             || (abs(pl_full_[i].z - pl_full_[i - 1].z) > 1e-7))) {
          pl_surf_.push_back(pl_full_[i]);
        }
      }
    }
  }
}

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
      continue;
    }
    double range = pl_orig.points[i].x * pl_orig.points[i].x
                   + pl_orig.points[i].y * pl_orig.points[i].y
                   + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < (blind_ * blind_) || range > det_range_ * det_range_
        || isnan(pl_orig.points[i].x) || isnan(pl_orig.points[i].y)
        || isnan(pl_orig.points[i].z))
      continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;

    added_pt.curvature = pl_orig.points[i].t * time_unit_scale_;

    pl_surf_.points.push_back(added_pt);
  }
}

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

  double omega_l = 0.361 * scan_rate_;
  std::vector<bool> is_first(n_scans_, true);
  std::vector<double> yaw_fp(n_scans_, 0.0);
  std::vector<float> time_last(n_scans_, 0.0);

  if (pl_orig.points[plsize - 1].time > 0) {
    given_offset_time_ = true;
  } else {
    given_offset_time_ = false;
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
    added_pt.curvature = pl_orig.points[i].time * time_unit_scale_;

    if (i % point_filter_num_ != 0 || std::isnan(added_pt.x)
        || std::isnan(added_pt.y) || std::isnan(added_pt.z))
      continue;

    if (!given_offset_time_) {
      int layer = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

      if (is_first[layer]) {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        added_pt.curvature = 0.0;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      if (yaw_angle < yaw_fp[layer]) {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      } else {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }
    }

    double dist = added_pt.getVector3fMap().squaredNorm();
    {
      if (dist > (blind_ * blind_) && dist < (det_range_ * det_range_)) {
        pl_surf_.points.push_back(added_pt);
      }
    }
  }
}

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

  double omega_l = 0.361 * scan_rate_;
  std::vector<bool> is_first(n_scans_, true);
  std::vector<double> yaw_fp(n_scans_, 0.0);
  std::vector<float> time_last(n_scans_, 0.0);

  if (pl_orig.points[plsize - 1].timestamp > 0) {
    given_offset_time_ = true;
  } else {
    given_offset_time_ = false;
  }

  double time_head = pl_orig.points[0].timestamp;

  for (std::size_t i = 0; i < plsize; i++) {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;

    added_pt.curvature = (pl_orig.points[i].timestamp - time_head)
                         * time_unit_scale_;

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

      if (yaw_angle <= yaw_fp[layer]) {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      } else {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.curvature < time_last[layer])
        added_pt.curvature += 360.0 / omega_l;

      time_last[layer] = added_pt.curvature;
    }

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
