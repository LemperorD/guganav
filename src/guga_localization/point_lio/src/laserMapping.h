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

struct MainLoopState {
  // ---- 控制标志 / 计数器 / 耗时统计 ----
  bool init_map = false;        ///< 地图已初始化
  bool flg_reset = false;       ///< 请求复位 (bag 回放等)
  bool flg_first_scan = true;   ///< 首次/复位后的第一帧
  int frame_num = 0;            ///< 已处理帧数
  int sleep_time = 0;           ///< 等待计数
  int time_log_counter = 0;     ///< 时间日志计数
  double solve_time = 0;        ///< 求解耗时 (s)
  double propag_time = 0;       ///< 状态传播耗时 (s)
  double update_time = 0;       ///< 更新耗时 (s)
  double aver_time_consu = 0;   ///< 平均单帧总耗时 (s)
  double aver_time_icp = 0;     ///< 平均配准耗时 (s)
  double aver_time_solve = 0;   ///< 平均求解耗时 (s)
  double aver_time_propag = 0;  ///< 平均传播耗时 (s)

  // ---- 工作缓存 (滤波器 / 消息 / 点云) ----
  pcl::VoxelGrid<PointType> downsize_filter_surf;  ///< 配准后降采样
  pcl::VoxelGrid<PointType> downsize_filter_map;   ///< 地图降采样
  V3D euler_cur;                                   ///< 欧拉角 (发布用)
  nav_msgs::msg::Path path;                        ///< 轨迹消息
  nav_msgs::msg::Odometry odom_aft_mapped;         ///< 里程计消息
  geometry_msgs::msg::PoseStamped msg_body_pose;   ///< 位姿消息
  PointCloudXYZI::Ptr feats_undistort =
      std::make_shared<PointCloudXYZI>();  ///< 去畸变特征点云
  PointCloudXYZI::Ptr init_feats_world =
      std::make_shared<PointCloudXYZI>();  ///< 初始化世界系点云
  PointCloudXYZI::Ptr pcl_wait_save =
      std::make_shared<PointCloudXYZI>();  ///< 待保存点云

  // ---- 滤波协方差 / 过程噪声 ----
  Eigen::Matrix<double, 24, 24> p_init;         ///< 初始协方差 (24维状态)
  Eigen::Matrix<double, 30, 30> p_init_output;  ///< 初始协方差 (30维状态)
  Eigen::Matrix<double, 24, 24> q_input;        ///< 过程噪声 (input 模式)
  Eigen::Matrix<double, 30, 30> q_output;       ///< 过程噪声 (output 模式)

  // ---- 位姿日志 ----
  std::string pos_log_dir;  ///< 位姿日志路径
  std::ofstream fp;         ///< 位姿日志文件

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LaserMappingNode : public rclcpp::Node {
public:
  /** @brief 节点构造: 以 "laserMapping" 为节点名初始化基类 */
  LaserMappingNode();

  /** @brief 节点入口: 初始化 + 主循环 (原 main) */
  int run();

private:
  // ==================== 成员变量 (原 main 局部) ====================
  Imu imu_;
  Lidar lidar_;
  rclcpp::executors::MultiThreadedExecutor
      executor_;         ///< 执行器 (主循环 spin_some)
  MainLoopState state_;  ///< 主循环状态
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_full_res_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_full_res_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_effect_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Rate rate_{500};  ///< 主循环频率

  // ==================== 私有成员函数 (原匿名namespace, 只由本节点调用)

  /** @brief 节点初始化: 参数 / 滤波器 / 日志 / 订阅发布 (由 run 调用) */
  void initialize();

  /** @brief 帧级初始化 (每轮主循环): 计时归零 + IMU 预处理 + 降采样/排序/分组
   *         + 地图就绪检查 + 量测准备
   * @return true  本帧可继续正常处理
   *         false IMU 初始化中或地图未就绪, 调用方应跳过本帧
   */
  bool prepareFrame();

  static PointCloudXYZI::Ptr loadPointcloudFromPcd(
      const std::string& file_path);
  /** @brief 将完整的 LIO 状态转储到日志文件 (输出到 state_.fp) */
  void dumpLioStatetoLog();

  static void pointBodyLidarToIMU(PointType const* pi, PointType* po);

  static void mapIncremental();

  void publishInitMap();

  void publishFrameWorld();

  void publishFrameBody();

  template <typename T>
  void setPosestamp(T& out);

  void publishOdometry();

  void publishPath();

  /** @brief 系统复位: 重置滤波器/里程计状态/地图 (bag 回放等场景) */
  void resetSystem();

  /** @brief 初始化地图: 累积世界系点云, 达到 init_map_size 后建图
   * (iVox/先验PCD)
   * @return true  地图已就绪, 本帧可继续正常处理
   *         false 初始化阶段 (本帧用于累积/建图, 调用方应跳过)
   */
  bool initMapState();

  static void preparePointMeasurements();

  /** @brief 帧尾: 计时收尾 + 发布输出 + 运行时位姿/耗时日志 */
  void publishAndLogFrame(double t0, double t1, double t2);

  /** @brief 帧内点处理 (2×2: 行=IMU 模式, 列=有无 LiDAR 点)
   * @tparam ImuAsInput true = kf_input (24维, IMU-as-input) / false =
   * kf_output (30维)
   * @param kf        对应滤波器 (kf_input / kf_output)
   * @param last_time 传播时间基准 (t_last / time_predict_last_const)
   * @param q         过程噪声 (state_.q_input / state_.q_output)
   */
  template <bool ImuAsInput, typename KF>
  void processFramePoints(KF& kf, double& last_time, auto& q);

  void initScan();

  void savePcd();
};