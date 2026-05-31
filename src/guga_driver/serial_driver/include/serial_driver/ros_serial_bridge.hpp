#ifndef ROS_SERIAL_BRIDGE_HPP
#define ROS_SERIAL_BRIDGE_HPP

#include "serial_driver_main.hpp"
#include "serial_driver_node.hpp"

namespace serial_driver
{

template<typename MsgType>
class RosMcuBridge
{
public: // 友元
  friend class SerialDriverNode;
  friend class SerialDriverMain;

public: // 构造和析构
  /**
   * @brief 构造函数
   * @param node SerialDriverNode的共享指针
   */
  explicit RosMcuBridge(
    const std::shared_ptr<SerialDriverNode> & node,
  );
  /**
   * @brief 析构函数
   */
  ~RosMcuBridge() override;
}

private: // 成员函数
  /**
   * @brief 连接MCU
   * @return 是否连接成功
   */
  bool connectMcu();
  /**
   * @brief 断开MCU连接
   */
  void disconnectMcu();
  /**
   * @brief 从MCU读取数据并发布ROS消息
   */
  void readFromMcuAndPublish();

} // namespace serial_driver

#endif // ROS_SERIAL_BRIDGE_HPP
