#include "serial_driver/serial_driver_node.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace serial_driver {

  SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
      : Node("serial_driver_node", options) {
    onConfigure();

    serial_driver_main_ = std::make_shared<SerialDriverMain>(port_name_,
                                                             baud_rate_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    const bool lidar_connected = false;
    std::cout << "Lidar connected: " << std::boolalpha << lidar_connected
              << '\n';

    bridge_twist_pc_ =
        std::make_shared<RosToSerialBridge<geometry_msgs::msg::Twist>>(
            this, "/cmd_vel",
            [this](const geometry_msgs::msg::Twist& msg) {
              return encodeTwist(msg);
            },
            [this](const uint8_t* data, size_t len) {
              serial_driver_main_->sendDataFrame(data, len);
            });

    bridge_yaw_mcu_ =
        std::make_shared<SerialToRosBridge<std_msgs::msg::Float32>>(
            this, "/serial/Yaw",
            [this](const uint8_t* payload) -> std_msgs::msg::Float32 {
              return decodeYaw(payload);
            },
            [this]() {
              return serial_driver_main_->receiveDataFrameSnapshot();
            });

    bridge_enemy_pos_mcu_ =
        std::make_shared<SerialToRosBridge<geometry_msgs::msg::Point>>(
            this, "/serial/EnemyPos",
            [](const uint8_t* payload) -> geometry_msgs::msg::Point {
              return decodeEnemyPos(payload);
            },
            [this]() {
              return serial_driver_main_->receiveDataFrameSnapshot();
            });

    gimbal_vision_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        [this]() { publishTransformGimbalVision(); });
    referee_rx_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), [this]() { publishRefereeData(); });

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

    gimbal_vision_timer_.reset();
    referee_rx_timer_.reset();

    bridge_twist_pc_.reset();
    bridge_yaw_mcu_.reset();
    // bridge_tes_speed_mcu_.reset();
    bridge_enemy_pos_mcu_.reset();

    // chassis_mode_sub_.reset();
    robot_status_pub_.reset();
    game_status_pub_.reset();
    rfid_status_pub_.reset();

    serial_driver_main_.reset();

    std::cout << "SerialDriverNode shutdown complete." << '\n';
  }

  // ==================== 参数加载 ====================

  void SerialDriverNode::onConfigure() {
    bool ok = shm_writer_.init("guga_shm", guga_ui::UiSlotId::GAME_STATUS);
    if (!ok) {
      RCLCPP_ERROR(logger_,
                   "ShmWriter init failed, UI game status display unavailable");
    }
    if (!shm_writer_yaw_.init("guga_shm", guga_ui::UiSlotId::YAW)) {
      RCLCPP_ERROR(logger_, "ShmWriter init failed, UI yaw display unavailable");
    }
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("vel_trans_scale", 40.0);
    this->declare_parameter<std::string>("lidar_ip", "192.168.1.2");
    this->declare_parameter<int>("lidar_port", 56360);

    this->get_parameter("port_name", port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("vel_trans_scale", vel_trans_scale_);
    this->get_parameter("lidar_ip", lidar_ip_);
    this->get_parameter("lidar_port", lidar_port_);
  }

  MotionPayload SerialDriverNode::encodeTwist(
      const geometry_msgs::msg::Twist& msg) const {
    const auto vx = static_cast<float>(vel_trans_scale_ * msg.linear.x);
    const auto vy = static_cast<float>(vel_trans_scale_ * msg.linear.y);
    const auto wz = static_cast<float>(msg.angular.z);

    MotionPayload payload{};
    payload.fill(0);  // 全部初始化为 0

    // 按头文件定义写入三个速度字段（VX=0, VY=4, WZ_NEG=8）
    SerialDriverMain::writeFloatLE(&payload[downlink_offset::VX], vx);
    SerialDriverMain::writeFloatLE(&payload[downlink_offset::VY], vy);
    SerialDriverMain::writeFloatLE(&payload[downlink_offset::WZ_NEG], wz);

    return payload;
  }

  std_msgs::msg::Float32 SerialDriverNode::decodeYaw(const uint8_t* payload) {
    std_msgs::msg::Float32 msg;
    msg.data = SerialDriverMain::readFloatLE(&payload[uplink_offset::YAW_DIFF]);
    yaw_diff_ = static_cast<double>(msg.data);
    guga_ui::UiYaw ui_yaw{};
    ui_yaw.yaw_diff = yaw_diff_;
    shm_writer_yaw_.write(&ui_yaw, sizeof(ui_yaw));
    return msg;
  }

  geometry_msgs::msg::Point SerialDriverNode::decodeEnemyPos(
      const uint8_t* payload) {
    geometry_msgs::msg::Point msg;
    msg.x = static_cast<double>(
        SerialDriverMain::readFloatLE(&payload[uplink_offset::ENEMY_X]));
    msg.y = static_cast<double>(
        SerialDriverMain::readFloatLE(&payload[uplink_offset::ENEMY_Y]));
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

  // ==================== 工具方法 ====================

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

  bool SerialDriverNode::isLidarConnected() const {
    const int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      std::cerr << "Socket creation failed\n";
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(lidar_port_);

    if (inet_pton(AF_INET, lidar_ip_.c_str(), &addr.sin_addr) <= 0) {
      std::cerr << "Invalid IP address: " << lidar_ip_ << '\n';
      close(sock);
      return false;
    }

    const int connect_result = connect(sock, reinterpret_cast<sockaddr*>(&addr),
                                       sizeof(addr));
    if (connect_result == 0) {
      close(sock);
      return true;
    }
    if (errno != EINPROGRESS) {
      close(sock);
      return false;
    }

    constexpr auto connect_timeout = std::chrono::milliseconds(500);
    const auto deadline = std::chrono::steady_clock::now() + connect_timeout;
    pollfd descriptor{sock, POLLOUT, 0};
    int poll_result{};
    do {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        close(sock);
        return false;
      }
      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      poll_result = poll(&descriptor, 1,
                         std::max(1, static_cast<int>(remaining.count())));
    } while (poll_result < 0 && errno == EINTR);

    if (poll_result <= 0) {
      close(sock);
      return false;
    }

    int socket_error{};
    socklen_t error_length = sizeof(socket_error);
    const bool connected = getsockopt(sock, SOL_SOCKET, SO_ERROR, &socket_error,
                                      &error_length)
                               == 0
                           && socket_error == 0;
    close(sock);
    return connected;
  }

  // ==================== 裁判系统 ====================

  void SerialDriverNode::publishRefereeData() {
    const auto payload_snapshot =
        serial_driver_main_->takeRefereeFrameSnapshot();
    if (!payload_snapshot.has_value()) {
      return;
    }
    const uint8_t* payload = payload_snapshot->data();

    // ---- 解析比赛进程（payload[0] 的高 4 位） ----
    uint8_t game_progress{};
    std::memcpy(&game_progress, &payload[referee_offset::GAME_PROGRESS],
                sizeof(uint8_t));
    game_progress = (game_progress >> 4);  // 取高 4 位
    guga_ui::UiGameStatus gp{};
    gp.elapsed_sec = this->now().seconds();
    gp.game_progress = game_progress;
    shm_writer_.write(&game_progress, sizeof(game_progress));

    // ---- 解析机器人状态 ----
    uint16_t current_hp{};
    uint16_t ammo17{};
    uint16_t heat1{};
    std::memcpy(&current_hp, &payload[referee_offset::CURRENT_HP],
                sizeof(uint16_t));
    std::memcpy(&ammo17, &payload[referee_offset::AMMO_17MM], sizeof(uint16_t));
    std::memcpy(&heat1, &payload[referee_offset::BARREL_HEAT],
                sizeof(uint16_t));

    // ---- 解析 RFID ----
    uint32_t rfid{};
    std::memcpy(&rfid, &payload[referee_offset::RFID_STATUS], sizeof(uint32_t));

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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::SerialDriverNode)
