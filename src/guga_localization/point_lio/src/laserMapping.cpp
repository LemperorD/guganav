/**
 * @file laserMapping.cpp
 * @brief Point-LIO 主处理流程 (laserMapping 节点)
 *
 * 这是 Point-LIO 的核心主循环, 负责:
 * - **节点初始化**: ROS2 订阅/发布/参数解析
 * - **主循环** (500Hz): 同步→预测→更新→建图→发布
 *
 * 主循环流水线:
 * @code
 *   sync_packages()            // 1. LiDAR-IMU 时间同步
 *   ↓
 *   p_imu->Process()           // 2. IMU 预积分 + 去畸变
 *   ↓
 *   downSizeFilterSurf         // 3. 体素降采样
 *   ↓
 *   EKF Predict + Update       // 4. 迭代卡尔曼 (逐点)
 *   ↓
 *   MapIncremental             // 5. 增量地图更新 (iVox)
 *   ↓
 *   publish_odometry/path等    // 6. 发布里程计/路径/点云/TF
 * @endcode
 *
 * 两种 EKF 模式:
 * - **IMU-as-input** (use_imu_as_input=true): IMU 驱动预测, 激光做量测更新
 * - **IMU-as-output** (use_imu_as_input=false, default): IMU 也作为量测,
 *   角速度和加速度本身被估计, 每帧同时做激光量测和 IMU 量测更新
 */

#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "li_initialization.h"

namespace laserMapping {
  bool flg_exit = false;

  /** @brief 去畸变后的点云 (IMU系) */
  PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());

  /** @brief 初始化阶段累积的世界系点云 */
  PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

  nav_msgs::msg::Path path;                       ///< 路径消息
  nav_msgs::msg::Odometry odomAftMapped;          ///< 里程计消息
  geometry_msgs::msg::PoseStamped msg_body_pose;  ///< 位姿消息 (用于路径)

  auto LOGGER = rclcpp::get_logger("laserMapping");
}  // namespace laserMapping
// ==================== 工具函数 ====================

/** @brief Ctrl+C 信号处理: 设置退出标志并通知条件变量 */
void SigHandle(int sig) {
  laserMapping::flg_exit = true;
  RCLCPP_WARN(laserMapping::LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

/** @brief 从 PCD 文件加载先验地图 */
PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string& file_path) {
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(laserMapping::LOGGER, "Couldn't read pcd file %s",
                 file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(laserMapping::LOGGER, "Loaded %zu points from %s",
              pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

/** @brief 将完整的 LIO 状态转储到日志文件
 *
 * 输出格式:
 *   time | euler(3) | pos(3) | omg(3) | vel(3) | acc(3) | bg(3) | ba(3) |
 * gravity(3)
 *
 * 根据 use_imu_as_input 选择不同的 KF 实例
 */
inline void dump_lio_state_to_log(FILE* fp) {
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1),
            kf_input.x_.pos(2));                 // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1),
            kf_input.x_.vel(2));                 // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1),
            kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1),
            kf_input.x_.ba(2));  // Bias_a
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
            kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1),
            kf_output.x_.pos(2));                // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1),
            kf_output.x_.vel(2));                // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // Acc
    fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1),
            kf_output.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1),
            kf_output.x_.ba(2));  // Bias_a
    fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0),
            kf_output.x_.gravity(1),
            kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

/**
 * @brief 雷达坐标系 → IMU 坐标系点变换
 *
 * 变换链: p_IMU = R_LI * p_LiDAR + T_LI
 * 根据 extrinsic_est_en 选择:
 *   - 在线估计: 使用 EKF 状态中的 offset_R_L_I / offset_T_L_I
 *   - 固定外参: 使用 YAML 中的 Lidar_R_wrt_IMU / Lidar_T_wrt_IMU
 */
void pointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar
                   + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar
                   + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

/**
 * @brief 增量地图更新: 将有效曲面点加入 iVox 局部地图
 *
 * 对每个世界坐标系下的曲面点:
 * 1. 检查 Nearest_Points[i] 是否已有 5 个近邻
 * 2. 如果有: 计算该点所在体素中心，判断是否已被地图覆盖
 *    - 如果附近已有地图点: 跳过 (避免冗余)
 *    - 否则: 加入 points_to_add
 * 3. 如果无: 直接加入 (新探索区域)
 * 4. 批量 AddPoints 到 iVox
 *
 * 该函数实现类似 "关键帧" 逻辑: 仅将地图中尚未覆盖的点加入。
 */
void MapIncremental() {
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (int i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType& point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector& points_near = Nearest_Points[i];

      Eigen::Vector3f center =
          ((point_world.getVector3fMap() / filter_size_map_min).array().floor()
           + 0.5)
          * filter_size_map_min;
      bool need_add = true;
      for (const auto x : points_near) {
        Eigen::Vector3f dis_2_center = x.getVector3fMap() - center;
        if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min
            && fabs(dis_2_center.y()) < 0.5 * filter_size_map_min
            && fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}

void publish_init_map(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
        pubLaserCloudFullRes) {
  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*laserMapping::init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
        pubLaserCloudFullRes) {
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *pcl_wait_save += *feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0
          && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_")
                              + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

void publish_frame_body(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
        pubLaserCloudFull_body) {
  int size = laserMapping::feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&laserMapping::feats_undistort->points[i],
                        &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T& out) {
  // Static variable, initialized to true, only effective on the first call
  static bool is_first_kf = true;

  auto set_output_from_kf = [&](const auto& kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    if (enable_prior_pcd && is_first_kf) {
      // Execute only on the first call
      kf_output.x_.pos(0) = init_pose[0];
      kf_output.x_.pos(1) = init_pose[1];
      kf_output.x_.pos(2) = init_pose[2];
      set_output_from_kf(kf_output);
      is_first_kf = false;  // Set is_first_kf to false after the first call
    } else {
      set_output_from_kf(kf_output);
    }
  } else {
    set_output_from_kf(kf_input);
  }
}

void publish_odometry(const rclcpp::Publisher<
                          nav_msgs::msg::Odometry>::SharedPtr& pubOdomAftMapped,
                      std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_br) {
  laserMapping::odomAftMapped.header.frame_id = "camera_init";
  laserMapping::odomAftMapped.child_frame_id = "body";
  if (publish_odometry_without_downsample) {
    laserMapping::odomAftMapped.header.stamp = get_ros_time(time_current);
  } else {
    laserMapping::odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  }
  set_posestamp(laserMapping::odomAftMapped.pose.pose);

  pubOdomAftMapped->publish(laserMapping::odomAftMapped);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "aft_mapped";
    transform.transform.translation.x =
        laserMapping::odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y =
        laserMapping::odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z =
        laserMapping::odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w =
        laserMapping::odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x =
        laserMapping::odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y =
        laserMapping::odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z =
        laserMapping::odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = laserMapping::odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);
  }
}

void publish_path(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
  set_posestamp(laserMapping::msg_body_pose.pose);
  // laserMapping::msg_body_pose.header.stamp = ros::Time::now();
  laserMapping::msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  laserMapping::msg_body_pose.header.frame_id = "camera_init";
  laserMapping::path.poses.emplace_back(laserMapping::msg_body_pose);
  pubPath->publish(laserMapping::path);
}

int main(int argc, char** argv) {
  int time_log_counter = 0;
  bool init_map = false;
  bool flg_first_scan = true;
  double match_time = 0;
  double solve_time = 0;
  double propag_time = 0;
  double update_time = 0;
  bool flg_reset = false;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  V3D euler_cur;
  int sleep_time = 0;

  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  std::cout << "lidar_type: " << lidar_type << '\n';
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  laserMapping::path.header.stamp = get_ros_time(lidar_end_time);
  laserMapping::path.header.frame_id = "camera_init";

  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0,
         aver_time_solve = 0, aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
                                filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output,
                                       h_model_output, h_model_IMU_output);

  Eigen::Matrix<double, 24, 24> P_init;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);

  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

  std::string pos_log_dir = std::string(ROOT_DIR) + "/Log/pos_log.txt";

  FILE* fp = fopen(pos_log_dir.c_str(), "w");
  open_file();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox;
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
          livox_pcl_cbk(msg);
        });
  } else {
    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          standard_pcl_cbk(msg);
        });
  }
  auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(), imu_cbk);

  auto pub_laser_cloud_full_res =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered",
                                                          20);
  auto pub_laser_cloud_full_res_body =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>(
          "cloud_registered_body", 20);
  auto pub_laser_cloud_effect =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 20);
  auto pub_laser_cloud_map =
      nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
  auto pub_odom_aft_mapped = nh->create_publisher<nav_msgs::msg::Odometry>(
      "aft_mapped_to_init", 20);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 20);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(500);
  while (rclcpp::ok()) {
    if (laserMapping::flg_exit)
      break;
    executor.spin_some();
    if (sync_packages(Measures)) {
      if (flg_reset) {
        RCLCPP_WARN(laserMapping::LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        laserMapping::feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);

          {
            while (Measures.lidar_beg_time
                   > get_time_sec(imu_next.header.stamp)) {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
        }
        G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1]
                           + gravity[2] * gravity[2]);
      }

      double t0, t1, t3, t5, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      // [Workflow 13] IMU 预处理，包括 IMU 有效性和饱和检查。
      t1 = omp_get_wtime();
      p_imu->Process(Measures, laserMapping::feats_undistort);
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(laserMapping::feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(),
             time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(),
             time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      }

      if (!p_imu->after_imu_init_) {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      if (!init_map) {
        feats_down_world->resize(laserMapping::feats_undistort->size());
        for (int i = 0; i < (int)laserMapping::feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(laserMapping::feats_undistort->points[i]),
                             &(feats_down_world->points[i]));
          }
        }
        for (const auto& point : *feats_down_world) {
          laserMapping::init_feats_world->points.emplace_back(point);
        }

        if (laserMapping::init_feats_world->size() >= (size_t)init_map_size) {
          if (enable_prior_pcd) {
            auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            ivox_->AddPoints(map_cloud->points);
          } else {
            ivox_->AddPoints(laserMapping::init_feats_world->points);
          }
          publish_init_map(pub_laser_cloud_map);
          laserMapping::init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        continue;
      }

      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      crossmat_list.reserve(feats_down_size);
      pbody_list.reserve(feats_down_size);

      // [Workflow 3] 将 LiDAR 点变换到 IMU 坐标系并准备量测量。
      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(feats_down_body->points[i].x,
                       feats_down_body->points[i].y,
                       feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en) {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }

      // [Workflow 2] LiDAR 点输入分支。
      if (!use_imu_as_input) {
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < (int)time_seq.size(); k++) {
            PointType& point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty())
                    break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x,
                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x,
                    imu_last.linear_acceleration.y,
                    imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            // [Workflow 1] 将状态传播到当前 LiDAR 点时间。
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp)
                              == get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp)
                         < time_predict_last_const
                     && !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty())
                    break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current
                               > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                angvel_avr << imu_next.angular_velocity.x,
                    imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x,
                    imu_next.linear_acceleration.y,
                    imu_next.linear_acceleration.z;

                double dt = get_time_sec(imu_next.header.stamp)
                            - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp)
                                  - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(laserMapping::LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            // [Workflow 4] 判断是否存在有效平面对应。
            // [Workflow 5] 残差、雅可比和量测协方差在 EKF 更新函数内计算。
            // [Workflow 6] 状态更新在 EKF 更新函数内完成。
            // [Workflow 7] 协方差更新在 EKF 更新函数内完成。
            if (!kf_output.update_iterated_dyn_share_modified()) {
              // [Workflow 9] 无有效平面对应时跳过当前点更新。
              idx = idx + time_seq[k];
              continue;
            }
            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20)
                         << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " "
                         << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " "
                         << kf_output.x_.omg.transpose() << " "
                         << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " "
                         << kf_output.x_.bg.transpose() << " "
                         << kf_output.x_.ba.transpose() << " "
                         << laserMapping::feats_undistort->points.size()
                         << '\n';
              }
            }

            // [Workflow 8] 将更新后的点变换到世界系。
            for (int j = 0; j < time_seq[k]; j++) {
              PointType& point_body_j = feats_down_body->points[idx + j + 1];
              PointType& point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
          }
        } else {
          // [Workflow 12] 当前时间段没有 LiDAR 点，仅处理 IMU 测量。
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current
                   && ((get_time_sec(imu_next.header.stamp)
                        < Measures.lidar_beg_time + lidar_time_inte))) {
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp)
                           < Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty())
                        break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x,
                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x,
                    imu_last.linear_acceleration.y,
                    imu_last.linear_acceleration.z;

                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x,
                    imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x,
                    imu_next.linear_acceleration.y,
                    imu_next.linear_acceleration.z;
                // [Workflow 14] IMU 残差、雅可比和量测协方差在该函数内计算。
                // [Workflow 15] IMU 状态更新在该函数内完成。
                // [Workflow 16] IMU 协方差更新在该函数内完成。
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        // [Workflow 11] IMU-as-input 分支。
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < (int)time_seq.size(); k++) {
            PointType& point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x,
                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x,
                    imu_last.linear_acceleration.y,
                    imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp)) {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x,
                  imu_last.angular_velocity.y, imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x,
                  imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp)
                              - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);

              if (imu_deque.empty())
                break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(laserMapping::LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            // [Workflow 4] 判断是否存在有效平面对应。
            // [Workflow 5] 残差、雅可比和量测协方差在 EKF 更新函数内计算。
            // [Workflow 6] 状态更新在 EKF 更新函数内完成。
            // [Workflow 7] 协方差更新在 EKF 更新函数内完成。
            if (!kf_input.update_iterated_dyn_share_modified()) {
              // [Workflow 9] 无有效平面对应时跳过当前点更新。
              idx = idx + time_seq[k];
              continue;
            }

            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20)
                         << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " "
                         << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " "
                         << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " "
                         << kf_input.x_.gravity.transpose() << " "
                         << laserMapping::feats_undistort->points.size()
                         << '\n';
              }
            }

            // [Workflow 8] 将更新后的点变换到世界系。
            for (int j = 0; j < time_seq[k]; j++) {
              PointType& point_body_j = feats_down_body->points[idx + j + 1];
              PointType& point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          // [Workflow 17] IMU-as-input 模式下的其他 IMU 处理分支。
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current
                   && ((get_time_sec(imu_next.header.stamp)
                        < Measures.lidar_beg_time + lidar_time_inte))) {
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp)
                           < Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty())
                        break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x,
                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x,
                    imu_last.linear_acceleration.y,
                    imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  time_update_last = get_time_sec(imu_next.header.stamp);
                }

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x,
                    imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x,
                    imu_next.linear_acceleration.y,
                    imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty())
                  break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // [Workflow 18] 发布当前状态、协方差对应的里程计和传感器结果。
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
      }

      t3 = omp_get_wtime();
      if (feats_down_size > 4) {
        if (enable_prior_pcd) {
          sleep_time++;
          if (sleep_time > 200) {
            MapIncremental();
          }
        } else {
          MapIncremental();
        }
      }
      t5 = omp_get_wtime();
      if (path_en)
        publish_path(pub_path);
      if (scan_pub_en || pcd_save_en)
        publish_frame_world(pub_laser_cloud_full_res);
      if (scan_pub_en && scan_body_pub_en)
        publish_frame_body(pub_laser_cloud_full_res_body);

      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num
                          + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num
                          + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num
                          + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num
                          + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num
                           + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] =
            laserMapping::feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter++;
        printf(
            "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: "
            "%0.6f ave solve: "
            "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: "
            "%0.6f propogate: %0.6f \n",
            t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3,
            aver_time_consu, aver_time_icp, aver_time_propag);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time
                     << " " << euler_cur.transpose() << " "
                     << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " "
                     << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " "
                     << kf_output.x_.gravity.transpose() << " "
                     << kf_output.x_.bg.transpose() << " "
                     << kf_output.x_.ba.transpose() << " "
                     << laserMapping::feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time
                     << " " << euler_cur.transpose() << " "
                     << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " "
                     << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " "
                     << kf_input.x_.gravity.transpose() << " "
                     << laserMapping::feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp);
      }
    }
    rate.sleep();
  }
  if (!pcl_wait_save->empty() && pcd_save_en) {
    auto t = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y_%m_%d-%H_%M_%S");
    std::string str_time = ss.str();

    string file_name = string("scans_" + str_time + ".pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }
  fout_out.close();
  fout_imu_pbp.close();
  return 0;
}
