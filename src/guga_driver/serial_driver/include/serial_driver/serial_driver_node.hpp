#ifndef SERIAL_DRIVER_NODE_HPP
#define SERIAL_DRIVER_NODE_HPP

#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/rfid_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

#include "serial_driver/ros_serial_bridge.hpp"
#include "serial_driver/serial_driver_main.hpp"

namespace serial_driver {

/**
 * @brief ROS2 节点层，封装 SerialDriverMain 并向上提供话题桥接、裁判数据发布、
 *        tf 广播等能力。
 *
 * 职责：
 *  - 从参数服务器读取串口配置（port_name, baud_rate 等）。
 *  - 持有并管理 SerialDriverMain 实例的生命周期。
 *  - 创建 4 路 RosMcuBridge 桥接通道（/cmd_vel, /serial/Yaw, /serial/TES_speed,
 *    /serial/EnemyPos）。
 *  - 定时发布裁判系统数据（RobotStatus, GameStatus, RfidStatus）。
 *  - 广播 gimbal_yaw_vision tf（由 odom→base_footprint 推导）。
 *  - 提供 DWA 滑动窗口滤波和速度坐标系变换工具方法。
 *  - 监听底盘模式切换（/chassis_mode）。
 */
class SerialDriverNode : public rclcpp::Node {
 public:
  /** @brief 构造函数，声明参数、打开串口、创建桥接器和定时器。 */
  explicit SerialDriverNode(const rclcpp::NodeOptions& options);

  /** @brief 析构函数，清理所有桥接器和串口资源。 */
  ~SerialDriverNode() override;

 private:
  // ---- 参数加载 ----
  /** @brief 声明并从参数服务器加载所有运行参数。 */
  void onConfigure();

  // ---- 编解码（供 RosMcuBridge 回调使用） ----

  /**
   * @brief 将 Twist 消息编码为 26 字节运动帧 payload。
   * @param msg 输入速度指令。
   * @return 指向 26 字节 payload 的指针（调用者不应释放）。
   */
  uint8_t* encodeTwist(const geometry_msgs::msg::Twist& msg);

  /**
   * @brief 从运动帧 payload 解码 yaw 角度差。
   * @param payload 运动帧 payload 指针。
   * @return 包含 yaw 角度差（弧度）的 Float32 消息。
   */
  std_msgs::msg::Float32 decodeYaw(const uint8_t* payload);

  /**
   * @brief 从运动帧 payload 解码 TES 线速度。
   * @param payload 运动帧 payload 指针。
   * @return 包含角速度 z 的 Twist 消息。
   */
  geometry_msgs::msg::Twist decodeTESspeed(const uint8_t* payload);

  /**
   * @brief 从运动帧 payload 解码敌方位置坐标。
   * @param payload 运动帧 payload 指针。
   * @return 包含 x, y 坐标的 Point 消息。
   */
  geometry_msgs::msg::Point decodeEnemyPos(const uint8_t* payload);

  // ---- tf 广播 ----

  /**
   * @brief 定时广播 gimbal_yaw_vision tf（30ms 周期）。
   *
   * 从 tf_buffer 查询 odom→base_footprint 变换，取 yaw 角后发布
   * base_footprint→gimbal_yaw_vision 的纯旋转 tf。
   */
  void publishTransformGimbalVision();

  /**
   * @brief 根据云台 yaw 角度广播 gimbal_yaw_odom→gimbal_yaw tf。
   * @param yaw 云台 yaw 角度（弧度）。
   */
  void publishTransformGimbalYaw(double yaw);

  // ---- 工具方法 ----

  /**
   * @brief 滑动窗口滤波器（DWA = Dynamic Window Average）。
   * @param sample 新样本值。
   * @return 滤波后的平均值。
   *
   * 使用 max_dwa_size_ 控制窗口长度，新样本入队、最旧样本出队后取均值。
   */
  double dwaFilter(double sample);

  /**
   * @brief 将速度从 gimbal_yaw 坐标系变换到 chassis 坐标系。
   * @param twist_in gimbal_yaw 系下的速度指令。
   * @param yaw_diff gimbal_yaw 与 chassis 之间的 yaw 角度差（弧度）。
   * @return chassis 系下的速度指令。
   */
  geometry_msgs::msg::Twist transformVelocityToChassis(
      const geometry_msgs::msg::Twist& twist_in, double yaw_diff);

  // ---- 裁判系统 ----

  /**
   * @brief 定时（20ms）从串口读取裁判系统帧并发布 RobotStatus/GameStatus/RfidStatus。
   */
  void publishRefereeData();

  /**
   * @brief 将裁判系统 RFID 原始 uint32_t 解析为 RfidStatus 消息。
   * @param rfid 裁判系统协议中的 rfid_status 字段。
   * @return 解析后的 RfidStatus 消息。
   */
  guga_interfaces::msg::RfidStatus rfid2ros(uint32_t rfid);

  // ---- 雷达检测 ----

  /**
   * @brief 通过 TCP 连接检测 Livox MID360 雷达是否在线。
   * @return true 表示雷达可达。
   *
   * 从配置文件读取雷达 IP 和点云端口，尝试 TCP connect（0.5 秒超时）。
   */
  [[nodiscard]] bool isLidarConnected() const;

 private:
  // 串口参数
  std::string port_name_{"/dev/ttyACM0"};
  int baud_rate_{115200};

  // 云台 yaw 相关
  double yaw_bias_{};
  double yaw_diff_{};
  int max_dwa_size_{15};
  std::deque<double> dwa_;

  // 速度变换
  double vel_trans_scale_{40.0};
  double angle_init_{};

  // 底盘模式
  uint8_t chassis_mode_{1};  // 默认 chassisFollowed

  // 雷达在线标志
  bool lidar_connected_{false};

  // 串口底层
  std::shared_ptr<SerialDriverMain> serial_driver_main_;

  // 四路消息桥接器
  std::shared_ptr<RosMcuBridge<geometry_msgs::msg::Twist>> bridge_twist_pc_;
  std::shared_ptr<RosMcuBridge<std_msgs::msg::Float32>> bridge_yaw_mcu_;
  std::shared_ptr<RosMcuBridge<geometry_msgs::msg::Twist>>
      bridge_tes_speed_mcu_;
  std::shared_ptr<RosMcuBridge<geometry_msgs::msg::Point>> bridge_enemy_pos_mcu_;

  // 定时器
  rclcpp::TimerBase::SharedPtr gimbal_vision_timer_;
  rclcpp::TimerBase::SharedPtr referee_rx_timer_;

  // tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 裁判系统发布者
  rclcpp::Publisher<guga_interfaces::msg::RobotStatus>::SharedPtr
      robot_status_pub_;
  rclcpp::Publisher<guga_interfaces::msg::GameStatus>::SharedPtr
      game_status_pub_;
  rclcpp::Publisher<guga_interfaces::msg::RfidStatus>::SharedPtr
      rfid_status_pub_;

  // 底盘模式订阅
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr chassis_mode_sub_;
};

}  // namespace serial_driver

#endif  // SERIAL_DRIVER_NODE_HPP
