#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "serial_driver/serial_driver_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<serial_driver::SerialDriverNode>(
      rclcpp::NodeOptions());

  // 多线程执行器，2 条线程：
  //  - /cmd_vel 订阅在 RosToSerialBridge 的专用 MutuallyExclusive 回调组
  //    （auto_add=true），与默认回调组（定时器、tf listener）分属不同组，
  //    组间可并行执行，定时器不会被订阅回调拖累；
  //  - 各组内部互斥，同一条回调不会并发执行（保护 encodeTwist 的滤波缓冲）；
  //  - 串口写阻塞由 bridge 的独立发送线程承担，不占用任何 executor 线程。
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
