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


#define RETURN0 0x00
#define RETURN0AND1 0x10


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
        pl_full_[i].x = msg->points[i].x;
        pl_full_[i].y = msg->points[i].y;
        pl_full_[i].z = msg->points[i].z;
        pl_full_[i].intensity = msg->points[i].reflectivity;

        pl_full_[i].curvature = msg->points[i].offset_time
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

    pl_surf_[i].curvature += (float)(rclcpp::Time(msg->header.stamp).seconds()
                                     * 1000)
                             - last_frame_end_time;
    pcl_cut.push_back(pl_surf_[i]);
    if (valid_num
        == (uint)(int((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
      cut_num++;
      time_lidar.push_back(last_frame_end_time);
      PointCloudXYZI::Ptr pcl_temp =
          std::make_shared<PointCloudXYZI>();
      *pcl_temp = pcl_cut;
      pcl_out.push_back(pcl_temp);

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
    double yaw_fp[MAX_LINE_NUM] = {0};
    double omega_l = 3.61;
    float time_last[MAX_LINE_NUM] = {0.0};

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
      added_pt.curvature = pl_orig.points[i].time * 1000.0;

      double dist = added_pt.getVector3fMap().squaredNorm();
      if (dist < blind_ * blind_ || dist > det_range_ * det_range_
          || isnan(added_pt.x) || isnan(added_pt.y)
          || isnan(added_pt.z)) {
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
      added_pt.curvature = pl_orig.points[i].t / 1e6;

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
                           * 1000;

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


        pl_full_[i].curvature = msg->points[i].offset_time
                                / float(1000000);


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
    added_pt.curvature = pl_orig.points[i].time
                         * time_unit_scale_;

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
