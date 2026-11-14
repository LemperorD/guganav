/**
 * @file laserMapping.h
 * @brief Point-LIO 节点类声明.
 *
 * 节点负责 ROS2 边界: 读取参数、建立订阅与发布器、驱动主循环.
 */

#include <atomic>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "point_lio/core/Lidar.h"
#include "point_lio/core/FrameProcessor.h"
#include "point_lio/OdomProcessor.h"
#include "point_lio/preprocess.h"

#include "point_lio/pcd/PriorMapManager.h"

#include "point_lio/ros2/pub/ProjectOutputPublisher.h"
#include "point_lio/ros2/pub/FramePublisher.h"
#include "point_lio/ros2/sub/SensorSubscriptions.h"

class LaserMappingNode : public rclcpp::Node {
public:
  /** @brief 节点构造: 以 "laserMapping" 为节点名初始化基类 */
  LaserMappingNode();
  ~LaserMappingNode() override;

private:
  void processIteration();

  void initializeParameters();
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

  /// @brief 收到定位状态: SUSPECT 开始累积, LOST 请求保存, FINE 清空缓存

  /** @brief 计算 PCD 输出路径; index > 0 时用于按 interval 分段保存 */

  /** @brief 原子写入 PCD, 可覆盖只读的先验地图文件 */

  // ==================== 成员变量 (原 main 局部) ====================
  DataHandler data_handler_;
  PointLioParams& config_ = data_handler_.config;
  PointLioStage stage_{PointLioStage::WAITINGFORDATA};
  PointLioStage prev_stage_{PointLioStage::WAITINGFORDATA};
  MainLoopState state_;  ///< 主循环状态
  FramePublisher frame_publisher_;
  OdomProcessor odom_projector_;
  PriorMapManager prior_map_;
  ProjectOutputPublisher project_publisher_;
  Preprocess preprocess_;
  SensorSubscriptions sensors_;
  FrameProcessor processor_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_state_pub_;

  // ==================== 项目约定输出 (原 loam_interface) ====================
  /// 与底盘对齐的 odom 系下的点云, 供 terrain_analysis / small_gicp /
  /// scan_to_sensor_frame 使用
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  /// 最近一帧里程计 (雷达里程计系), 供换算到 odom 系后发布
};
