#include "serial_driver/serial_driver_node.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <fstream>
#include <iostream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using json = nlohmann::json;

namespace serial_driver {

  SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
      : Node("serial_driver_node", options),
        lidar_connected_(isLidarConnected()) {
    onConfigure();

    serial_driver_main_ = std::make_shared<SerialDriverMain>(port_name_,
                                                             baud_rate_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    std::cout << "Lidar connected: " << std::boolalpha << lidar_connected_
              << '\n';

    // TODO 话题名是否正确待检查
    bridge_twist_pc_ =
        std::make_shared<RosMcuBridge<geometry_msgs::msg::Twist>>(
            this, "/cmd_vel", true,  // ros_to_serial = PC → MCU
            [this](const geometry_msgs::msg::Twist& msg) {
              return encodeTwist(msg);
            },
            nullptr,  // 不需要 decoder（PC→MCU 方向）
            [this](const uint8_t* data, size_t len) {
              serial_driver_main_->sendDataFrame(data, len);
            },
            nullptr);  // 不需要 receiver（PC→MCU 方向）

    // --- /serial/Yaw (MCU → PC): 从下位机接收云台 yaw 角度差 ---
    bridge_yaw_mcu_ = std::make_shared<RosMcuBridge<std_msgs::msg::Float32>>(
        this, "/serial/Yaw", false,  // ros_to_serial = MCU → PC
        nullptr,                     // encoder 不需要（MCU→PC 方向）
        [this](const uint8_t* payload) -> std_msgs::msg::Float32 {
          return decodeYaw(payload);
        },
        nullptr,  // sender 不需要（MCU→PC 方向）
        [this]() -> const uint8_t* {
          return serial_driver_main_->receiveDataFrame();
        });

    // --- /serial/TES_speed (MCU → PC): 从下位机接收 TES 速度 ---
    bridge_tes_speed_mcu_ =
        std::make_shared<RosMcuBridge<geometry_msgs::msg::Twist>>(
            this, "/serial/TES_speed", false,  // MCU → PC
            nullptr,
            [](const uint8_t* payload) -> geometry_msgs::msg::Twist {
              return decodeTESspeed(payload);
            },
            nullptr,
            [this]() -> const uint8_t* {
              return serial_driver_main_->receiveDataFrame();
            });

    // --- /serial/EnemyPos (MCU → PC): 从下位机接收敌方坐标 ---
    bridge_enemy_pos_mcu_ =
        std::make_shared<RosMcuBridge<geometry_msgs::msg::Point>>(
            this, "/serial/EnemyPos", false,  // MCU → PC
            nullptr,
            [this](const uint8_t* payload) -> geometry_msgs::msg::Point {
              return decodeEnemyPos(payload);
            },
            nullptr,
            [this]() -> const uint8_t* {
              return serial_driver_main_->receiveDataFrame();
            });

    gimbal_vision_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        [this]() { publishTransformGimbalVision(); });
    referee_rx_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), [this]() { publishRefereeData(); });

    chassis_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "chassis_mode", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          chassis_mode_ = static_cast<uint8_t>(msg->data);
        });

    robot_status_pub_ =
        this->create_publisher<guga_interfaces::msg::RobotStatus>(
            "/referee/robot_status", 10);
    game_status_pub_ = this->create_publisher<guga_interfaces::msg::GameStatus>(
        "/referee/game_status", 10);
    rfid_status_pub_ = this->create_publisher<guga_interfaces::msg::RfidStatus>(
        "/referee/rfid_status", 10);
  }

  SerialDriverNode::~SerialDriverNode() {
    std::cout << "Shutting down SerialDriverNode..." << '\n';

    // 先停止定时器
    gimbal_vision_timer_.reset();
    referee_rx_timer_.reset();

    // 清理桥接器（停止接收线程）
    bridge_twist_pc_.reset();
    bridge_yaw_mcu_.reset();
    bridge_tes_speed_mcu_.reset();
    bridge_enemy_pos_mcu_.reset();

    // 清理订阅和发布者
    chassis_mode_sub_.reset();
    robot_status_pub_.reset();
    game_status_pub_.reset();
    rfid_status_pub_.reset();

    // 最后清理串口底层
    serial_driver_main_.reset();

    std::cout << "SerialDriverNode shutdown complete." << '\n';
  }

  // ==================== 参数加载 ====================

  void SerialDriverNode::onConfigure() {
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("Yaw_bias", 0.0);
    this->declare_parameter<int>("max_dwa_size", 15);
    this->declare_parameter<double>("vel_trans_scale", 40.0);

    this->get_parameter("port_name", port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("Yaw_bias", yaw_bias_);
    this->get_parameter("max_dwa_size", max_dwa_size_);
    this->get_parameter("vel_trans_scale", vel_trans_scale_);
  }

  std::array<uint8_t, SerialDriverNode::PAYLOAD_SIZE>
  SerialDriverNode::encodeTwist(const geometry_msgs::msg::Twist& msg) const {
    geometry_msgs::msg::Twist twist_chassis = transformVelocityToChassis(
        msg, yaw_diff_ * M_PI / 180.0);

    const auto vx = static_cast<float>(vel_trans_scale_ * msg.linear.x);
    const auto vy = static_cast<float>(vel_trans_scale_ * msg.linear.y);
    const auto wz = static_cast<float>(msg.angular.z);
    const auto vx_y = static_cast<float>(vel_trans_scale_
                                         * twist_chassis.linear.x);
    const auto vy_y = static_cast<float>(vel_trans_scale_
                                         * twist_chassis.linear.y);

    std::array<uint8_t, SerialDriverNode::PAYLOAD_SIZE> payload{};
    payload[OFFSET_CHASSIS_MODE] = chassis_mode_;
    SerialDriverMain::writeFloatLE(&payload[OFFSET_ANGLE_INIT],
                                   static_cast<float>(angle_init_));
    payload[OFFSET_FLAG] = 1;
    SerialDriverMain::writeFloatLE(&payload[OFFSET_VX_Y], vx_y);
    SerialDriverMain::writeFloatLE(&payload[OFFSET_VY_Y], vy_y);
    SerialDriverMain::writeFloatLE(&payload[OFFSET_VX], vx);
    SerialDriverMain::writeFloatLE(&payload[OFFSET_VY], vy);
    SerialDriverMain::writeFloatLE(&payload[OFFSET_WZ_NEG], -wz);

    return payload;
  }
  // TODO 串口协议
  std_msgs::msg::Float32 SerialDriverNode::decodeYaw(const uint8_t* payload) {
    // payload[7..10] 存储 yaw 角度差（float LE）
    std_msgs::msg::Float32 msg;
    std::memcpy(&yaw_diff_, payload + 7, sizeof(float));
    msg.data = static_cast<float>(yaw_diff_);
    return msg;
  }

  geometry_msgs::msg::Twist SerialDriverNode::decodeTESspeed(
      const uint8_t* payload) {
    // payload[3..6] 存储角速度 z（float LE，由 readFloatLE 解析）
    geometry_msgs::msg::Twist msg;
    msg.angular.z = static_cast<double>(
        SerialDriverMain::readFloatLE(&payload[3]));
    return msg;
  }

  geometry_msgs::msg::Point SerialDriverNode::decodeEnemyPos(
      const uint8_t* payload) {
    // payload[11..12] int16_t x, payload[13..14] int16_t y（小端序）
    geometry_msgs::msg::Point msg;
    int16_t x{};
    int16_t y{};
    std::memcpy(&x, payload + 11, sizeof(int16_t));
    std::memcpy(&y, payload + 13, sizeof(int16_t));
    msg.x = static_cast<double>(x);
    msg.y = static_cast<double>(y);
    return msg;
  }

  // ==================== tf 广播 ====================

  void SerialDriverNode::publishTransformGimbalVision() {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      transform_stamped = tf_buffer_->lookupTransform("odom", "base_footprint",
                                                      tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      if (isLidarConnected()) {
        // 雷达在线但 tf 不可用，说明定位可能出问题
        std::cerr << "Could not get transform: " << ex.what() << '\n';
      }
      return;
    }

    // 提取 yaw 角
    tf2::Quaternion q(transform_stamped.transform.rotation.x,
                      transform_stamped.transform.rotation.y,
                      transform_stamped.transform.rotation.z,
                      transform_stamped.transform.rotation.w);

    double roll{};
    double pitch{};
    double yaw{};
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 发布 base_footprint → gimbal_yaw_vision（纯旋转，取反 yaw）
    geometry_msgs::msg::TransformStamped gimbal_tf;
    gimbal_tf.header.stamp = this->get_clock()->now();
    gimbal_tf.header.frame_id = "base_footprint";
    gimbal_tf.child_frame_id = "gimbal_yaw_vision";

    gimbal_tf.transform.translation.x = 0.0;
    gimbal_tf.transform.translation.y = 0.0;
    gimbal_tf.transform.translation.z = 0.0;

    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0.0, 0.0, -yaw);

    gimbal_tf.transform.rotation.x = q_yaw.x();
    gimbal_tf.transform.rotation.y = q_yaw.y();
    gimbal_tf.transform.rotation.z = q_yaw.z();
    gimbal_tf.transform.rotation.w = q_yaw.w();

    tf_broadcaster_->sendTransform(gimbal_tf);
  }

  void SerialDriverNode::publishTransformGimbalYaw(double yaw) {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "gimbal_yaw_odom";
    tf_msg.child_frame_id = "gimbal_yaw";

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    // DWA 滤波 + yaw 偏置补偿
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -dwaFilter(yaw) + yaw_bias_);

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ==================== 工具方法 ====================

  double SerialDriverNode::dwaFilter(double sample) {
    dwa_.push_back(sample);
    if (dwa_.size() > static_cast<size_t>(max_dwa_size_)) {
      dwa_.pop_front();
    }

    double sum{};
    for (double x : dwa_) {
      sum += x;
    }
    return dwa_.empty() ? sample : (sum / static_cast<double>(dwa_.size()));
  }

  geometry_msgs::msg::Twist SerialDriverNode::transformVelocityToChassis(
      const geometry_msgs::msg::Twist& twist_in, double yaw_diff) {
    geometry_msgs::msg::Twist out;
    double cos = std::cos(yaw_diff);
    double sin = std::sin(yaw_diff);

    out.linear.x = (twist_in.linear.x * cos) - (twist_in.linear.y * sin);
    out.linear.y = (twist_in.linear.x * sin) + (twist_in.linear.y * cos);
    out.angular.z = twist_in.angular.z;
    return out;
  }

  // ==================== 雷达检测 ====================

  bool SerialDriverNode::isLidarConnected() {
    // 读取 MID360 配置文件
    // TODO 硬编码处理
    std::ifstream f("/home/ld/nav2_ws/config/mid360_user_config.json");
    if (!f.is_open()) {
      std::cerr << "Cannot open Lidar config file\n";
      return false;
    }

    json config;
    try {
      f >> config;
    } catch (...) {
      std::cerr << "JSON parse failed\n";
      return false;
    }

    std::string lidar_ip = config["lidar_configs"][0]["ip"];
    int point_port = config["MID360"]["lidar_net_info"]["point_data_port"];

    // 通过 TCP connect 测试雷达可达性
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      std::cerr << "Socket creation failed\n";
      return false;
    }

    // 0.5 秒超时
    struct timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(point_port);

    if (inet_pton(AF_INET, lidar_ip.c_str(), &addr.sin_addr) <= 0) {
      std::cerr << "Invalid IP address\n";
      close(sock);
      return false;
    }

    int ret = connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    close(sock);

    return (ret == 0);
  }

  // ==================== 裁判系统 ====================

  void SerialDriverNode::publishRefereeData() {
    const uint8_t* payload = serial_driver_main_->receiveRefereeFrame();
    if (payload == nullptr) {
      std::cerr << "Failed to receive referee frame." << '\n';
      return;
    }

    // ---- 解析比赛进程（payload[0] 的高 4 位） ----
    uint8_t game_progress{};
    std::memcpy(&game_progress, payload, sizeof(uint8_t));
    game_progress = (game_progress >> 4);  // 取高 4 位

    // ---- 解析机器人状态 ----
    uint16_t current_hp{};
    uint16_t ammo17{};
    uint16_t heat1{};
    std::memcpy(&current_hp, payload + 1, sizeof(uint16_t));
    std::memcpy(&ammo17, payload + 3, sizeof(uint16_t));
    std::memcpy(&heat1, payload + 5, sizeof(uint16_t));

    // ---- 解析 RFID ----
    uint32_t rfid{};
    std::memcpy(&rfid, payload + 9, sizeof(uint32_t));

    // ---- 发布机器人状态 ----
    guga_interfaces::msg::RobotStatus robot_status;
    robot_status.current_hp = current_hp;
    robot_status.projectile_allowance_17mm = ammo17;
    robot_status.shooter_17mm_1_barrel_heat = heat1;
    robot_status_pub_->publish(robot_status);

    // ---- 发布比赛状态 ----
    guga_interfaces::msg::GameStatus game_status;
    game_status.game_progress = game_progress;
    game_status_pub_->publish(game_status);

    // ---- 发布 RFID 状态 ----
    guga_interfaces::msg::RfidStatus rfid_status;
    rfid_status = rfid2ros(rfid);
    rfid_status_pub_->publish(rfid_status);

    // 消费帧标志
    serial_driver_main_->clearRefereeFrameFlag();
  }

  guga_interfaces::msg::RfidStatus SerialDriverNode::rfid2ros(uint32_t rfid) {
    guga_interfaces::msg::RfidStatus status;

    // 裁判系统协议 V1.7.0 0x0209: RFID 状态位映射
    status.base_gain_point = (rfid >> 0) & 0x01;
    status.central_highland_gain_point = (rfid >> 1) & 0x01;
    status.enemy_central_highland_gain_point = (rfid >> 2) & 0x01;
    status.friendly_trapezoidal_highland_gain_point = (rfid >> 3) & 0x01;
    status.enemy_trapezoidal_highland_gain_point = (rfid >> 4) & 0x01;
    status.friendly_fly_ramp_front_gain_point = (rfid >> 5) & 0x01;
    status.friendly_fly_ramp_back_gain_point = (rfid >> 6) & 0x01;
    status.enemy_fly_ramp_front_gain_point = (rfid >> 7) & 0x01;
    status.enemy_fly_ramp_back_gain_point = (rfid >> 8) & 0x01;
    status.friendly_central_highland_lower_gain_point = (rfid >> 9) & 0x01;
    status.friendly_central_highland_upper_gain_point = (rfid >> 10) & 0x01;
    status.enemy_central_highland_lower_gain_point = (rfid >> 11) & 0x01;
    status.enemy_central_highland_upper_gain_point = (rfid >> 12) & 0x01;
    status.friendly_highway_lower_gain_point = (rfid >> 13) & 0x01;
    status.friendly_highway_upper_gain_point = (rfid >> 14) & 0x01;
    status.enemy_highway_lower_gain_point = (rfid >> 15) & 0x01;
    status.enemy_highway_upper_gain_point = (rfid >> 16) & 0x01;
    status.friendly_fortress_gain_point = (rfid >> 17) & 0x01;
    status.friendly_outpost_gain_point = (rfid >> 18) & 0x01;
    status.friendly_supply_zone_non_exchange = (rfid >> 19) & 0x01;
    status.friendly_supply_zone_exchange = (rfid >> 20) & 0x01;
    status.friendly_big_resource_island = (rfid >> 21) & 0x01;
    status.enemy_big_resource_island = (rfid >> 22) & 0x01;
    status.center_gain_point = (rfid >> 23) & 0x01;

    return status;
  }

}  // namespace serial_driver

// 注册为 rclcpp component，可通过 composition 加载
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::SerialDriverNode)
