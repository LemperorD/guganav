#include <atomic>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "point_lio/core/Lidar.h"
#include "point_lio/core/Filter.h"
#include "point_lio/core/FrameProcessor.h"
#include "point_lio/core/ProcessingState.h"

class LaserMappingNode : public rclcpp::Node {
public:
  /** @brief 节点构造: 以 "laserMapping" 为节点名初始化基类 */
  LaserMappingNode();
  ~LaserMappingNode() override;

private:
  // ==================== 成员变量 (原 main 局部) ====================
  Imu imu_;
  Lidar lidar_;
  PointLioParams config_;
  PointLioStage stage_{PointLioStage::WAITINGFORDATA};
  int pcd_index_{0};
  int pcd_scan_count_{0};
  /// @brief 导航模式下收到 GICP LOST 信号后才开始累积/保存 PCD
  std::atomic<bool> pcd_save_armed_{false};
  MainLoopState state_;  ///< 主循环状态
  FrameProcessor processor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  /// @brief GICP LOST 触发信号 (std_msgs/Bool)
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pcd_save_trigger_;
  /// @brief GICP 定位状态 (std_msgs/UInt8): 0=INITIAL 1=FINE 2=SUSPECT 3=LOST
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gicp_state_;
  /// @brief 先验地图重载服务客户端, 新 PCD 写盘后调用
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_client_;
  /// @brief 最近一次收到的定位状态
  std::atomic<uint8_t> gicp_state_{0};
  /// @brief 是否已把 LOST 触发过一次保存请求 (每次 FINE 后复位)
  bool pcd_save_on_lost_done_{false};
  /// @brief 主循环看到该标志后执行保存 (回调只置标志, 不碰点云)
  std::atomic<bool> pcd_save_requested_{false};
  /// @brief SUSPECT/LOST 累计计时 (只在主线程读写)
  bool suspect_tracking_{false};
  double suspect_accum_sec_{0.0};
  rclcpp::Time suspect_mark_;
  /// @brief 首次 LOST 已按覆盖方式写过一次地图; 之后改为增量合并
  bool first_rebuild_done_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_full_res_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_full_res_body_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_laser_cloud_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ==================== 项目约定输出 (原 loam_interface) ====================
  /// 与底盘对齐的 odom 系下的点云, 供 terrain_analysis / small_gicp /
  /// scan_to_sensor_frame 使用
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_registered_scan_;
  /// 同一个 odom 系下的雷达里程计
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lidar_odometry_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  /// 安装变换: odom 系 = R * (雷达里程计系) + t, 取自 TF 的 base_frame <-
  /// lidar_frame
  Eigen::Matrix3d odom_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d odom_translation_{Eigen::Vector3d::Zero()};
  bool odom_extrinsic_ready_{false};
  /// 最近一帧里程计 (雷达里程计系), 供换算到 odom 系后发布
  nav_msgs::msg::Odometry last_odometry_;
  bool has_last_odometry_{false};

  void processIteration();
  void createSensorSubscriptions();

  void initializeSensors();
  void initializeMappingState();
  void initializeRos2Interfaces();

  /** @brief 轮次初始化: 同步传感器数据、处理首帧并准备当前帧
   * @return true 当前轮次已准备好进入 ESKF 处理
   *         false 尚未同步到数据或 IMU/地图仍在初始化
   */

  /** @brief 帧级初始化 (每轮主循环): 计时归零 + IMU 预处理 + 降采样/排序/分组
   *         + 地图就绪检查 + 量测准备
   * @return true  本帧可继续正常处理
   *         false IMU 初始化中或地图未就绪, 调用方应跳过本帧
   */

  void publishFrameWorld();

  void publishFrameBody();

  template <typename T>
  void setPosestamp(T& out);

  void publishPath();

  /** @brief 初始化地图: 累积世界系点云, 达到 init_map_size 后建图
   * (iVox/先验PCD)
   * @return true  地图已就绪, 本帧可继续正常处理
   *         false 初始化阶段 (本帧用于累积/建图, 调用方应跳过)
   */

  /** @brief 帧尾: 发布输出 + 运行时位姿日志 */
  void publishFrameOutputs();

  // ==================== 项目约定输出 ====================
  /** @brief 解析安装变换: 优先用参数, 否则查一次 TF base_frame <- lidar_frame
   * @return true 变换可用
   */
  bool resolveOdomExtrinsic();

  /** @brief 按 odom 系 Z 阈值判断世界系点是否保留 */
  [[nodiscard]] bool pointPassesOdomZFilter(
      const Eigen::Vector3d& point_world) const;

  /** @brief 发布 odom 系下的点云与雷达里程计 (原 loam_interface 的输出) */
  void publishProjectOutputs();

  void savePendingPcd();
  void savePcd();
  /// @brief 收到定位状态: SUSPECT 开始累积, LOST 请求保存, FINE 清空缓存
  void gicpStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  /// @brief 把累积点云写到 pcd_save_path, 然后调用重载服务
  void savePcdOnLost();
  /// @brief 主循环里累计 SUSPECT/LOST 时长, 达到阈值后增量更新地图并重载
  void maybeUpdateMapOnSuspect();
  /// @brief 旧地图 + 累积点云合并后体素下采样再原子写回
  bool mergeMapAndWrite();
  /// @brief 写盘前把旧地图复制为带时间戳的备份
  void backupPriorMap();
  /// @brief 异步调用先验地图重载服务
  void requestPriorMapReload();

  /** @brief PCD 保存是否已启用 (导航模式还需等待 LOST 触发) */
  [[nodiscard]] bool isPcdSaveActive() const;

  /** @brief 计算 PCD 输出路径; index > 0 时用于按 interval 分段保存 */
  [[nodiscard]] std::string pcdOutputPath(int index) const;

  /** @brief 原子写入 PCD, 可覆盖只读的先验地图文件 */
  bool writePcd(const std::string& path, const PointCloudXYZI& cloud);
};
