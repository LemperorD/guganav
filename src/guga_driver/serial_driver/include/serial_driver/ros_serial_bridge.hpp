#ifndef SERIAL_DRIVER_ROS_SERIAL_BRIDGE_HPP
#define SERIAL_DRIVER_ROS_SERIAL_BRIDGE_HPP

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace serial_driver {

/**
 * @brief ROS ↔ MCU 双向消息桥接模板类。
 *
 * 将 ROS 话题与串口数据帧之间建立编码/解码通道，支持两种方向：
 *  - PC → MCU（ros_to_serial = true）：订阅 ROS 话题，收到消息后编码并调用
 *    串口发送回调。
 *  - MCU → PC（ros_to_serial = false）：通过内部轮询线程从串口接收回调获取
 *    原始 payload，解码后发布到 ROS 话题。
 *
 * 设计思路：不持有串口对象的引用，而是通过 std::function 回调注入编解码和
 * 收发逻辑，解耦 ROS 层与串口物理层。
 *
 * @tparam RosMsgT ROS 消息类型，如 geometry_msgs::msg::Twist。
 */
template <typename RosMsgT>
class RosMcuBridge {
 public:
  /** @brief 编码回调：ROS 消息 → 串口 payload 字节数组。 */
  using EncoderFunc = std::function<const uint8_t*(const RosMsgT&)>;

  /** @brief 解码回调：串口 payload 字节数组 → ROS 消息。 */
  using DecoderFunc = std::function<RosMsgT(const uint8_t*)>;

  /** @brief 串口发送回调：payload + 长度 → 写入串口。 */
  using SerialSendFunc = std::function<void(const uint8_t*, size_t)>;

  /** @brief 串口接收回调：无参，返回最新的 payload 指针（可能为 nullptr）。 */
  using SerialRecvFunc = std::function<const uint8_t*()>;

  /**
   * @brief 构造并启动桥接通道。
   *
   * @param node ROS2 节点指针（用于创建 pub/sub）。
   * @param ros_topic_name 要桥接的 ROS 话题名。
   * @param ros_to_serial true: ROS→MCU 方向; false: MCU→ROS 方向。
   * @param encoder 编码函数（ros_to_serial=true 时必须提供）。
   * @param decoder 解码函数（ros_to_serial=false 时必须提供）。
   * @param serial_sender 串口发送回调（ros_to_serial=true 时必须提供）。
   * @param serial_receiver 串口接收回调（ros_to_serial=false 时必须提供）。
   * @param recv_hz MCU→ROS 模式下接收线程的轮询频率（默认 200Hz）。
   */
  RosMcuBridge(rclcpp::Node* node, const std::string& ros_topic_name,
               bool ros_to_serial, EncoderFunc encoder, DecoderFunc decoder,
               SerialSendFunc serial_sender, SerialRecvFunc serial_receiver,
               double recv_hz = 200.0)
      : node_(node),
        encoder_(std::move(encoder)),
        decoder_(std::move(decoder)),
        serial_sender_(std::move(serial_sender)),
        serial_receiver_(std::move(serial_receiver)) {
    auto qos = rclcpp::QoS(10);

    if (ros_to_serial) {
      // PC → MCU: 订阅 ROS 话题，收到消息后编码 → 串口发送
      sub_ = node_->template create_subscription<RosMsgT>(
          ros_topic_name, qos,
          [this](const typename RosMsgT::SharedPtr msg) {
            const uint8_t* payload = encoder_(*msg);
            serial_sender_(payload, 26);  // 26 字节运动帧 payload
          });
    } else {
      // MCU → PC: 内部线程轮询串口接收 → 解码 → 发布到 ROS 话题
      pub_ = node_->template create_publisher<RosMsgT>(ros_topic_name, qos);
      recv_thread_ = std::thread([this, recv_hz]() {
        rclcpp::Rate rate(recv_hz);
        while (rclcpp::ok()) {
          const uint8_t* payload = serial_receiver_();
          if (payload != nullptr) {
            RosMsgT msg = decoder_(payload);
            pub_->publish(msg);
          }
          rate.sleep();
        }
      });
    }
  }

  /** @brief 析构，等待接收线程退出。 */
  ~RosMcuBridge() {
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }

 private:
  rclcpp::Node* node_;

  EncoderFunc encoder_;
  DecoderFunc decoder_;
  SerialSendFunc serial_sender_;
  SerialRecvFunc serial_receiver_;

  typename rclcpp::Publisher<RosMsgT>::SharedPtr pub_;
  typename rclcpp::Subscription<RosMsgT>::SharedPtr sub_;
  std::thread recv_thread_;
};

}  // namespace serial_driver

#endif  // SERIAL_DRIVER_ROS_SERIAL_BRIDGE_HPP
