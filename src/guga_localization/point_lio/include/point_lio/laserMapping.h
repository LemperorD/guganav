#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "point_lio/Lidar.h"
#include "point_lio/Synchronizer.h"
#include "point_lio/Filter.h"
#include "point_lio/FrameProcessor.h"

struct MainLoopState {
  int sleep_time = 0;  ///< 等待计数

  // ---- 工作缓存 (滤波器 / 消息 / 点云) ----
  pcl::VoxelGrid<PointType> downsize_filter_surf;  ///< 配准后降采样
  nav_msgs::msg::Path path;                        ///< 轨迹消息
  nav_msgs::msg::Odometry odom_aft_mapped;         ///< 里程计消息
  geometry_msgs::msg::PoseStamped msg_body_pose;   ///< 位姿消息
  PointCloudXYZI::Ptr feats_undistort =
      std::make_shared<PointCloudXYZI>();  ///< 去畸变特征点云
  PointCloudXYZI::Ptr init_feats_world =
      std::make_shared<PointCloudXYZI>();  ///< 初始化世界系点云
  PointCloudXYZI::Ptr pcl_wait_save =
      std::make_shared<PointCloudXYZI>();  ///< 待保存点云

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LaserMappingNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /** @brief 节点构造: 以 "laserMapping" 为节点名初始化基类 */
  LaserMappingNode();
  ~LaserMappingNode() override;

private:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  // ==================== 成员变量 (原 main 局部) ====================
  Imu imu_;
  Lidar lidar_;
  Synchronizer synchronizer_;
  Filter filter_;
  PointLioParams config_;
  bool parameters_loaded_{false};
  PointLioStage stage_{PointLioStage::WAITINGFORDATA};
  bool is_first_frame_{true};
  double lidar_end_time_{0.0};
  int pcd_index_{0};
  int pcd_scan_count_{0};
  double time_update_last_{0.0};
  double time_predict_last_const_{0.0};
  double t_last_{0.0};
  MeasureGroup measures_;
  MainLoopState state_;  ///< 主循环状态
  FrameProcessor processor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_full_res_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_full_res_body_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_map_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr
      pub_odom_aft_mapped_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      pub_path_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
  void processIteration();
  void createSensorSubscriptions();
  void destroySensorSubscriptions();

  void initializeSensors();
  void initializeMappingState();
  void initializeFilter();
  void initializeRos2Interfaces();

  /** @brief 轮次初始化: 同步传感器数据、处理首帧并准备当前帧
   * @return true 当前轮次已准备好进入 ESKF 处理
   *         false 尚未同步到数据或 IMU/地图仍在初始化
   */
  bool initializeIteration();

  /** @brief 帧级初始化 (每轮主循环): 计时归零 + IMU 预处理 + 降采样/排序/分组
   *         + 地图就绪检查 + 量测准备
   * @return true  本帧可继续正常处理
   *         false IMU 初始化中或地图未就绪, 调用方应跳过本帧
   */
  bool prepareFrame();

  PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string& file_path);
  void pointBodyLidarToIMU(PointType const* pi, PointType* po) const;

  void mapIncremental() const;

  void publishInitMap();

  void publishFrameWorld();

  void publishFrameBody();

  template <typename T>
  void setPosestamp(T& out);

  void publishOdometry();

  void publishPath();

  /** @brief 初始化地图: 累积世界系点云, 达到 init_map_size 后建图
   * (iVox/先验PCD)
   * @return true  地图已就绪, 本帧可继续正常处理
   *         false 初始化阶段 (本帧用于累积/建图, 调用方应跳过)
   */
  bool initMapState();

  void preparePointMeasurements() const;

  /** @brief 帧尾: 发布输出 + 运行时位姿日志 */
  void publishFrameOutputs();

  /** @brief 帧内点处理 (2×2: 行=IMU 模式, 列=有无 LiDAR 点)
   * @tparam ImuAsInput true = input filter (24维) / false = output filter
   * (30维)
   * @param kf        当前模式对应的滤波器
   * @param last_time 传播时间基准 (t_last / time_predict_last_const)
   * @param q         对应滤波器的过程噪声
   */
  template <bool ImuAsInput, typename KF>
  void processFramePoints(KF& kf, double& last_time, auto& q);

  void initScan();

  void savePendingPcd();
  void savePcd();
};
