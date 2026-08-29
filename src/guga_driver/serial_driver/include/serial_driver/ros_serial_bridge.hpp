#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "serial_driver/br_protocol_types.hpp"

namespace serial_driver {

/**
 * @brief PC → MCU 单向消息桥接模板类。
 *
 * 订阅 ROS 话题，收到消息后编码并入队，由内部独立发送线程从有界队列取出
 * payload 并调用串口发送回调。
 * 设计思路：不持有串口对象的引用，而是通过 std::function 回调注入编码和
 * 发送逻辑，解耦 ROS 层与串口物理层。订阅回调只做编码和入队（微秒级），
 * 串口写入（可能阻塞）全部在独立发送线程上执行，不占用 executor 线程。
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
   * @param queue_capacity 发送队列容量（默认 1）。队列满时丢弃最旧帧，
   *        保证待发队列始终“最新优先”；容量 1 即只保留最新一帧。
   */
  RosToSerialBridge(rclcpp::Node* node, const std::string& ros_topic_name,
                    EncoderFunc encoder, SerialSendFunc serial_sender,
                    size_t queue_capacity = 1)
      : node_(node),
        encoder_(std::move(encoder)),
        serial_sender_(std::move(serial_sender)),
        queue_capacity_(std::max<size_t>(1, queue_capacity))
  {
    auto qos = rclcpp::QoS(10);
    sub_ = node_->template create_subscription<RosMsgT>(
        ros_topic_name, qos, [this](const typename RosMsgT::SharedPtr msg) {
          if (stop_.load(std::memory_order_relaxed)) {
            return;  // 析构已开始，不再入队
          }
          MotionPayload payload = encoder_(*msg);
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() >= queue_capacity_) {
              queue_.pop_front();  // 满则丢最旧，保持最新优先
            }
            queue_.push_back(std::move(payload));
          }
          cv_.notify_one();
        });
    send_thread_ = std::thread([this]() {
      while (true) {
        MotionPayload payload;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait(lock, [this]() { return stop_ || !queue_.empty(); });
          if (queue_.empty()) {  // stop_ 且队列已清空
            break;
          }
          payload = std::move(queue_.front());
          queue_.pop_front();
        }
        // 串口 I/O 在锁外执行；阻塞只影响发送线程自身
        serial_sender_(payload.data(), payload.size());
      }
    });
  }

  ~RosToSerialBridge() {
    stop_.store(true, std::memory_order_relaxed);
    cv_.notify_all();
    if (send_thread_.joinable()) {
      send_thread_.join();  // 排空队列中剩余的待发帧后退出
    }
  }
  RosToSerialBridge(const RosToSerialBridge&) = delete;
  RosToSerialBridge(RosToSerialBridge&&) = delete;
  RosToSerialBridge& operator=(const RosToSerialBridge&) = delete;
  RosToSerialBridge& operator=(const RosToSerialBridge&&) = delete;

private:
  rclcpp::Node* node_;
  EncoderFunc encoder_;
  SerialSendFunc serial_sender_;
  typename rclcpp::Subscription<RosMsgT>::SharedPtr sub_;
  size_t queue_capacity_;
  std::deque<MotionPayload> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> stop_{false};
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
