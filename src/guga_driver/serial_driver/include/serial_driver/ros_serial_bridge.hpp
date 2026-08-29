#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "serial_driver/br_protocol_types.hpp"

namespace serial_driver {

  /**
   * @brief PC → MCU 单向消息桥接模板类。
   *
   * 订阅 ROS 话题，收到消息后编码并调用串口发送回调。
   * 设计思路：不持有串口对象的引用，而是通过 std::function 回调注入编码和
   * 发送逻辑，解耦 ROS 层与串口物理层。
   *
   * @tparam RosMsgT ROS 消息类型，如 geometry_msgs::msg::Twist。
   */
  template <typename RosMsgT>
  class RosToSerialBridge {
  public:
    /** @brief 编码回调：ROS 消息 → 串口 payload 字节数组。 */
    using EncoderFunc = std::function<MotionPayload(const RosMsgT&)>;

    /** @brief 串口发送回调：payload + 长度 → 写入串口。 */
    using SerialSendFunc = std::function<void(const uint8_t*, size_t)>;

    /**
     * @brief 构造并启动 PC→MCU 桥接通道。
     *
     * @param node ROS2 节点指针（用于创建 subscriber）。
     * @param ros_topic_name 要订阅的 ROS 话题名。
     * @param encoder 编码函数（ROS 消息 → 串口 payload）。
     * @param serial_sender 串口发送回调。
     */
    RosToSerialBridge(rclcpp::Node* node, const std::string& ros_topic_name,
                      EncoderFunc encoder, SerialSendFunc serial_sender)
        : node_(node),
          encoder_(std::move(encoder)),
          serial_sender_(std::move(serial_sender)) {
      auto qos = rclcpp::QoS(10);
      sub_ = node_->template create_subscription<RosMsgT>(
          ros_topic_name, qos, [this](const typename RosMsgT::SharedPtr msg) {
            const auto payload = encoder_(*msg);
            serial_sender_(payload.data(), payload.size());
          });
    }

    ~RosToSerialBridge() = default;
    RosToSerialBridge(const RosToSerialBridge&) = delete;
    RosToSerialBridge(RosToSerialBridge&&) = delete;
    RosToSerialBridge& operator=(const RosToSerialBridge&) = delete;
    RosToSerialBridge& operator=(const RosToSerialBridge&&) = delete;

  private:
    rclcpp::Node* node_;
    EncoderFunc encoder_;
    SerialSendFunc serial_sender_;
    typename rclcpp::Subscription<RosMsgT>::SharedPtr sub_;
    std::thread send_thread_;
  };

  /**
   * @brief MCU → PC 单向消息桥接模板类。
   *
   * 通过内部轮询线程从串口接收回调获取原始 payload，解码后发布到 ROS 话题。
   * 设计思路：不持有串口对象的引用，而是通过 std::function 回调注入解码和
   * 接收逻辑，解耦 ROS 层与串口物理层。
   *
   * @tparam RosMsgT ROS 消息类型，如 std_msgs::msg::Float32。
   */
  template <typename RosMsgT>
  class SerialToRosBridge {
  public:
    /** @brief 解码回调：串口 payload 字节数组 → ROS 消息。 */
    using DecoderFunc = std::function<RosMsgT(const uint8_t*)>;

    /** @brief 串口接收回调：无参，返回最新 payload 的值快照。 */
    using SerialRecvFunc = std::function<MotionPayload()>;

    /**
     * @brief 构造并启动 MCU→PC 桥接通道。
     *
     * @param node ROS2 节点指针（用于创建 publisher）。
     * @param ros_topic_name 要发布的 ROS 话题名。
     * @param decoder 解码函数（串口 payload → ROS 消息）。
     * @param serial_receiver 串口接收回调。
     * @param recv_hz 接收线程的轮询频率（默认 200Hz）。
     */
    SerialToRosBridge(rclcpp::Node* node, const std::string& ros_topic_name,
                      DecoderFunc decoder, SerialRecvFunc serial_receiver,
                      double recv_hz = 200.0)
        : node_(node),
          decoder_(std::move(decoder)),
          serial_receiver_(std::move(serial_receiver)) {
      auto qos = rclcpp::QoS(10);
      pub_ = node_->template create_publisher<RosMsgT>(ros_topic_name, qos);
      recv_thread_ = std::thread([this, recv_hz]() {
        rclcpp::Rate rate(recv_hz);
        while (rclcpp::ok()) {
          const auto payload = serial_receiver_();
          RosMsgT msg = decoder_(payload.data());
          pub_->publish(msg);
          rate.sleep();
        }
      });
    }

    SerialToRosBridge(const SerialToRosBridge&) = delete;
    SerialToRosBridge(SerialToRosBridge&&) = delete;
    SerialToRosBridge& operator=(const SerialToRosBridge&) = delete;
    SerialToRosBridge& operator=(const SerialToRosBridge&&) = delete;

    ~SerialToRosBridge() {
      if (recv_thread_.joinable()) {
        recv_thread_.join();
      }
    }

  private:
    rclcpp::Node* node_;
    DecoderFunc decoder_;
    SerialRecvFunc serial_receiver_;
    typename rclcpp::Publisher<RosMsgT>::SharedPtr pub_;
    std::thread recv_thread_;
  };

}  // namespace serial_driver
