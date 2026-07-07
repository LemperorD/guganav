/**
 * @file parameters.cpp
 * @brief 全局参数定义和 readParameters() 的实现
 *
 * 该文件声明了所有全局变量并提供了从 ROS2 参数服务器读取 YAML 配置的功能。
 * 参数读取使用 try-catch 保护，确保在参数类型不匹配时打印错误信息。
 *
 * 参数分三大类:
 * - **common**: 话题名、合帧/切帧设置、时间偏移
 * - **mapping**: 建图相关 (IMU协方差、饱和值、滤波、重力、外参)
 * - **preprocess**: 预处理 (LiDAR类型、线数、扫描频率、盲区)
 * - **odometry/publish/pcd_save**: 输出相关 (发布开关、路径、PCD)
 */

#include "parameters.h"

// ==================== 帧控制标志 ====================
bool is_first_frame = true;              ///< 首帧标志 (用于跳过第一帧的预测)
double lidar_end_time = 0.0;             ///< 当前帧最晚点时间戳
double first_lidar_time = 0.0;           ///< 第一帧雷达时间 (用于相对计时)
double time_con = 0.0;                   ///< 合帧模式起始时间
double last_timestamp_lidar = -1.0;      ///< 上帧雷达时间 (初始 -1 表示未收到数据)
double last_timestamp_imu = -1.0;        ///< 上帧 IMU 时间 (初始 -1 表示未收到数据)
int pcd_index = 0;                       ///< PCD 保存序号

// ==================== iVox 地图参数 ====================
IVoxType::Options ivox_options_;         ///< iVox 体素配置 (分辨率、近邻方式)
int ivox_nearby_type = 6;               ///< 默认 6 邻域

// ==================== 外参 ====================
std::vector<double> extrinT(3, 0.0);     ///< LiDAR→IMU 平移外参 [x, y, z]
std::vector<double> extrinR(9, 0.0);     ///< LiDAR→IMU 旋转外参 (3x3 行主序)

// ==================== EKF 状态 ====================
state_input state_in;                    ///< IMU-as-input 模式状态
state_output state_out;                  ///< IMU-as-output 模式状态

// ==================== ROS2 话题名 ====================
std::string lid_topic, imu_topic;

// ==================== 模式开关 (默认值) ====================
bool prop_at_freq_of_imu = true;        ///< 默认: 按 IMU 频率传播
bool check_satu = true;                 ///< 默认: 检查 IMU 饱和
bool con_frame = false;                 ///< 默认: 不合帧
bool cut_frame = false;                 ///< 默认: 不切帧
bool use_imu_as_input = false;          ///< 默认: IMU-as-output (30维)
bool space_down_sample = true;          ///< 默认: 再次空间降采样
bool publish_odometry_without_downsample = false;  ///< 默认: 逐帧发布里程计

// ==================== 地图初始化 ====================
int init_map_size = 10;                 ///< 初始地图最少点数
int con_frame_num = 1;                  ///< 合帧数

// ==================== 匹配阈值 ====================
double match_s = 81;                    ///< 点面匹配 Mahalanobis 距离阈值
double satu_acc;                        ///< 加速度计饱和值 (从 YAML 读取)
double satu_gyro;                       ///< 陀螺仪饱和值 (从 YAML 读取)
double cut_frame_time_interval = 0.1;   ///< 切帧时间间隔 (秒)

// ==================== 平面提取 ====================
float plane_thr = 0.1f;                 ///< 平面残差阈值
double filter_size_surf_min = 0.5;      ///< 曲面降采样分辨率
double filter_size_map_min = 0.5;       ///< 地图降采样分辨率
double fov_deg = 180;                   ///< 默认全向 FOV

// ==================== 检测范围 ====================
float DET_RANGE = 450;                  ///< 最大检测距离 450m

// ==================== IMU 参数 ====================
bool imu_en = true;                     ///< 默认启用 IMU
double imu_time_inte = 0.005;           ///< IMU 时间步长 5ms
double laser_point_cov = 0.01;          ///< 激光点量测噪声 0.01
double acc_norm;                        ///< 加速度归一化因子
double vel_cov;                         ///< output 模式速度噪声
double acc_cov_input;                   ///< input 模式加速度噪声
double gyr_cov_input;                   ///< input 模式陀螺仪噪声
double gyr_cov_output;                  ///< output 模式陀螺仪噪声
double acc_cov_output;                  ///< output 模式加速度噪声
double b_gyr_cov;                       ///< 陀螺仪零偏随机游走噪声
double b_acc_cov;                       ///< 加速度计零偏随机游走噪声
double imu_meas_acc_cov;                ///< IMU 加速度量测噪声
double imu_meas_omg_cov;                ///< IMU 角速度量测噪声

// ==================== LiDAR 参数 ====================
int lidar_type;                         ///< 雷达类型 (从 YAML 读取)
int pcd_save_interval;                  ///< PCD 保存间隔

// ==================== 重力 ====================
std::vector<double> gravity_init;       ///< 初始重力向量
std::vector<double> gravity;            ///< 当前重力向量

// ==================== 输出开关 ====================
bool runtime_pos_log;                   ///< 运行时位姿日志
bool pcd_save_en;                       ///< PCD 保存
bool path_en;                           ///< 路径发布
bool extrinsic_est_en = true;           ///< 外参在线估计 (默认: 启用)
bool scan_pub_en;                       ///< 配准点云发布
bool scan_body_pub_en;                  ///< IMU系点云发布
bool tf_send_en;                        ///< TF 发布

// ==================== 处理器实例 ====================
shared_ptr<Preprocess> p_pre;           ///< 点云预处理模块
shared_ptr<ImuProcess> p_imu;           ///< IMU 处理模块

// ==================== 时间戳管理 ====================
double time_update_last = 0.0;          ///< 上帧协方差更新时间
double time_current = 0.0;             ///< 当前处理时间
double time_predict_last_const = 0.0;   ///< 上帧状态预测时间
double t_last = 0.0;                   ///< input 模式的上帧时间
double time_diff_lidar_to_imu = 0.0;    ///< 雷达→IMU 时间偏移

// ==================== 先验 PCD 地图 ====================
bool enable_prior_pcd;                 ///< 是否启用先验地图
string prior_pcd_map_path;             ///< 先验地图路径
std::vector<double> init_pose;         ///< 初始位姿

// ==================== 帧参数 ====================
double lidar_time_inte = 0.1;           ///< LiDAR 帧间隔 (默认 10Hz)
double first_imu_time = 0.0;            ///< 首帧 IMU 时间戳
int cut_frame_num = 1;                  ///< 切帧数
int orig_odom_freq = 10;                ///< 原始里程计发布频率
double online_refine_time = 20.0;       ///< 在线精标定时间 (秒)
bool cut_frame_init = false;            ///< 初始化切帧 (默认: 关)

// ==================== 数据同步 ====================
MeasureGroup Measures;                  ///< 当前处理的测量组

// ==================== 调试日志 ====================
ofstream fout_out;                      ///< 位姿输出文件
ofstream fout_imu_pbp;                  ///< IMU 逐点输出文件

void readParameters(std::shared_ptr<rclcpp::Node> & nh)
{
  // 初始化 Preprocess 和 ImuProcess 实例
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  try {
    // ==================== 模式开关 ====================
    nh->declare_parameter<bool>("prop_at_freq_of_imu", true);
    nh->get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);

    nh->declare_parameter<bool>("use_imu_as_input", false);
    nh->get_parameter("use_imu_as_input", use_imu_as_input);

    nh->declare_parameter<bool>("check_satu", true);
    nh->get_parameter("check_satu", check_satu);

    nh->declare_parameter<int>("init_map_size", 100);
    nh->get_parameter("init_map_size", init_map_size);

    nh->declare_parameter<bool>("space_down_sample", true);
    nh->get_parameter("space_down_sample", space_down_sample);

    // ==================== mapping 参数 — IMU 饱和 ====================
    nh->declare_parameter<double>("mapping.satu_acc", 3.0);
    nh->get_parameter("mapping.satu_acc", satu_acc);

    nh->declare_parameter<double>("mapping.satu_gyro", 35.0);
    nh->get_parameter("mapping.satu_gyro", satu_gyro);

    nh->declare_parameter<double>("mapping.acc_norm", 1.0);
    nh->get_parameter("mapping.acc_norm", acc_norm);

    // ==================== mapping 参数 — 平面提取 ====================
    nh->declare_parameter<float>("mapping.plane_thr", 0.05f);
    nh->get_parameter("mapping.plane_thr", plane_thr);

    nh->declare_parameter<int>("point_filter_num", 2);
    nh->get_parameter("point_filter_num", p_pre->point_filter_num);

    // ==================== common 参数 — 话题名 ====================
    nh->declare_parameter<std::string>("common.lid_topic", ".livox.lidar");
    nh->get_parameter("common.lid_topic", lid_topic);

    nh->declare_parameter<std::string>("common.imu_topic", ".livox.imu");
    nh->get_parameter("common.imu_topic", imu_topic);

    // ==================== common 参数 — 合帧/切帧 ====================
    nh->declare_parameter<bool>("common.con_frame", false);
    nh->get_parameter("common.con_frame", con_frame);

    nh->declare_parameter<int>("common.con_frame_num", 1);
    nh->get_parameter("common.con_frame_num", con_frame_num);

    nh->declare_parameter<bool>("common.cut_frame", false);
    nh->get_parameter("common.cut_frame", cut_frame);

    nh->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
    nh->get_parameter("common.cut_frame_time_interval", cut_frame_time_interval);

    nh->declare_parameter<double>("common.time_diff_lidar_to_imu", 0.0);
    nh->get_parameter("common.time_diff_lidar_to_imu", time_diff_lidar_to_imu);

    // ==================== prior_pcd 参数 — 先验地图 ====================
    nh->declare_parameter<bool>("prior_pcd.enable", false);
    nh->get_parameter("prior_pcd.enable", enable_prior_pcd);

    nh->declare_parameter<string>("prior_pcd.prior_pcd_map_path", "");
    nh->get_parameter("prior_pcd.prior_pcd_map_path", prior_pcd_map_path);

    nh->declare_parameter<std::vector<double>>("prior_pcd.init_pose", std::vector<double>());
    nh->get_parameter("prior_pcd.init_pose", init_pose);

    // ==================== 滤波参数 ====================
    nh->declare_parameter<double>("filter_size_surf", 0.5);
    nh->get_parameter("filter_size_surf", filter_size_surf_min);

    nh->declare_parameter<double>("filter_size_map", 0.5);
    nh->get_parameter("filter_size_map", filter_size_map_min);

    nh->declare_parameter<float>("mapping.det_range", 300.f);
    nh->get_parameter("mapping.det_range", DET_RANGE);

    nh->declare_parameter<double>("mapping.fov_degree", 180);
    nh->get_parameter("mapping.fov_degree", fov_deg);

    // ==================== mapping 参数 — IMU 功能开关 ====================
    nh->declare_parameter<bool>("mapping.imu_en", true);
    nh->get_parameter("mapping.imu_en", imu_en);

    nh->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    nh->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);

    nh->declare_parameter<double>("mapping.imu_time_inte", 0.005);
    nh->get_parameter("mapping.imu_time_inte", imu_time_inte);

    // ==================== mapping 参数 — 噪声协方差 ====================
    nh->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
    nh->get_parameter("mapping.lidar_meas_cov", laser_point_cov);

    nh->declare_parameter<double>("mapping.acc_cov_input", 0.1);
    nh->get_parameter("mapping.acc_cov_input", acc_cov_input);

    nh->declare_parameter<double>("mapping.vel_cov", 20);
    nh->get_parameter("mapping.vel_cov", vel_cov);

    nh->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
    nh->get_parameter("mapping.gyr_cov_input", gyr_cov_input);

    nh->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
    nh->get_parameter("mapping.gyr_cov_output", gyr_cov_output);

    nh->declare_parameter<double>("mapping.acc_cov_output", 0.1);
    nh->get_parameter("mapping.acc_cov_output", acc_cov_output);

    nh->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    nh->get_parameter("mapping.b_gyr_cov", b_gyr_cov);

    nh->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    nh->get_parameter("mapping.b_acc_cov", b_acc_cov);

    nh->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
    nh->get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);

    nh->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
    nh->get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);

    // ==================== preprocess 参数 — 雷达配置 ====================
    nh->declare_parameter<double>("preprocess.blind", 1.0);
    nh->get_parameter("preprocess.blind", p_pre->blind);

    nh->declare_parameter<int>("preprocess.lidar_type", 1);
    nh->get_parameter("preprocess.lidar_type", lidar_type);

    nh->declare_parameter<int>("preprocess.scan_line", 16);
    nh->get_parameter("preprocess.scan_line", p_pre->N_SCANS);

    nh->declare_parameter<int>("preprocess.scan_rate", 10);
    nh->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);

    nh->declare_parameter<int>("preprocess.timestamp_unit", 1);
    nh->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);

    nh->declare_parameter<double>("mapping.match_s", 81);
    nh->get_parameter("mapping.match_s", match_s);

    // ==================== mapping 参数 — 重力 ====================
    nh->declare_parameter<std::vector<double>>("mapping.gravity", std::vector<double>());
    nh->get_parameter("mapping.gravity", gravity);

    nh->declare_parameter<std::vector<double>>("mapping.gravity_init", std::vector<double>());
    nh->get_parameter("mapping.gravity_init", gravity_init);

    // ==================== mapping 参数 — 外参 ====================
    nh->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    nh->get_parameter("mapping.extrinsic_T", extrinT);

    nh->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());
    nh->get_parameter("mapping.extrinsic_R", extrinR);

    // ==================== odometry/publish 参数 ====================
    nh->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
    nh->get_parameter(
      "odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);

    nh->declare_parameter<bool>("publish.path_en", true);
    nh->get_parameter("publish.path_en", path_en);

    nh->declare_parameter<bool>("publish.scan_publish_en", true);
    nh->get_parameter("publish.scan_publish_en", scan_pub_en);

    nh->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    nh->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);

    nh->declare_parameter<bool>("publish.tf_send_en", true);
    nh->get_parameter("publish.tf_send_en", tf_send_en);

    nh->declare_parameter<bool>("runtime_pos_log_enable", false);
    nh->get_parameter("runtime_pos_log_enable", runtime_pos_log);

    // ==================== pcd_save 参数 ====================
    nh->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    nh->get_parameter("pcd_save.pcd_save_en", pcd_save_en);

    nh->declare_parameter<int>("pcd_save.interval", -1);
    nh->get_parameter("pcd_save.interval", pcd_save_interval);

    nh->declare_parameter<double>("mapping.lidar_time_inte", 0.1);
    nh->get_parameter("mapping.lidar_time_inte", lidar_time_inte);

    // ==================== iVox 网格参数 ====================
    nh->declare_parameter<float>("mapping.ivox_grid_resolution", 0.2);
    nh->get_parameter("mapping.ivox_grid_resolution", ivox_options_.resolution_);

    nh->declare_parameter<int>("ivox_nearby_type", 18);
    nh->get_parameter("ivox_nearby_type", ivox_nearby_type);
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(nh->get_logger(), "Parameter type exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(nh->get_logger(), "Exception: %s", e.what());
  }

  // 根据 ivox_nearby_type 设置 iVox 近邻搜索模式
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;    ///< 仅体素中心
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;   ///< 面邻域 (6个)
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;  ///< 面+边邻域 (18个) [推荐]
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;  ///< 面+边+角邻域 (26个)
  } else {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;  ///< 未知值默认 18 邻域
  }
  // 将 YAML 重力向量传递给 IMU 处理模块
  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

/**
 * @brief SO(3) 流形 → ZYX 欧拉角转换
 *
 * 利用 MTK::SO3 类型的 operator() 提取旋转矩阵元素，
 * 然后按照 ZYX 顺序解算欧拉角。
 * 处理万向节死锁情况 (sy≈0 时 yaw=0)。
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 & rot)
{
  double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;  ///< 万向节死锁阈值
  double x, y, z;
  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));  ///< roll  = atan2(R32, R33)
    y = atan2(-rot(2, 0), sy);         ///< pitch = atan2(-R31, √(R11²+R21²))
    z = atan2(rot(1, 0), rot(0, 0));  ///< yaw   = atan2(R21, R11)
  } else {
    // 万向节死锁: 设 yaw = 0, 通过 R12,R13 计算 roll
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<double, 3, 1> ang(x, y, z);
  return ang;
}

/** @brief 打开调试日志文件 (位姿和 IMU) */
void open_file()
{
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
  fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
  if (fout_out && fout_imu_pbp)
    std::cout << "~~~~" << ROOT_DIR << " file opened" << '\n';
  else
    std::cout << "~~~~" << ROOT_DIR << " doesn't exist" << '\n';
}

/**
 * @brief 重置 IMU-as-input 模式的卡尔曼协方差矩阵
 *
 * 协方差分块:
 * - [0:15]:   位置/姿态/速度/外参 = 0.1*I
 * - [15:21]:  速度部分 = 0.001*I
 * - [21:24]:  重力方向 = 0.0001*I
 */
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
  P_init = MD(24, 24)::Identity() * 0.1;
  P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
}

/**
 * @brief 重置 IMU-as-output 模式的卡尔曼协方差矩阵
 *
 * 协方差分块:
 * - [0:24]:   位置/姿态/速度/外参/角速度/加速度 = 0.01*I
 * - [21:24]:  重力方向 = 0.0001*I
 * - [24:30]:  零偏 = 0.001*I
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
  P_init_output = MD(30, 30)::Identity() * 0.01;
  P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
  P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
}