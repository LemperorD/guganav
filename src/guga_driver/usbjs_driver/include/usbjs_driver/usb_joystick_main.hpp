#ifndef USB_JOYSTICK_MAIN_HPP
#define USB_JOYSTICK_MAIN_HPP

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <thread>
#include <iostream>
#include <sstream>
#include <chrono>
#include <atomic>

namespace usbjs_driver
{

class UsbJoystick
{
public: // 构造和析构
  /**
   * @brief 构造函数
   * @param file_name 设备文件地址
   */
  explicit UsbJoystick(const char *file_name);

  /**
   * @brief 析构函数
   */
  ~UsbJoystick();

public: // 公共接口
  /**
   * @brief 获取设备文件描述符
   * @return 文件描述符
   */
  int get_fd() const { return js_fd_; }

  /**
   * @brief 获取轴状态
   * @param id 轴的ID
   * @return 轴的状态
   */
  int get_axis_state(int id) {
    if (id >= 0 && id < (int)axis_state_.size()) return axis_state_[id];
    else return 0;
  };

  /**
   * @brief 获取按钮状态
   * @param id 按钮的ID
   * @return 按钮的状态
   */
  int get_button_state(int id) {
    if (id >= 0 && id < (int)button_state_.size()) return button_state_[id];
    else return 0;
  };

private: // 打开设备
  /**
   * @brief 打开USB手柄设备
   * @param file_name 设备文件地址
   * @return 文件描述符
   */
  int js_open(const char *file_name);

private: // 轮询线程
  /**
   * @brief 轮询线程
   */
  void timerThread();

  /**
   * @brief 轮询回调函数
   */
  void timerCallback();

  /**
   * @brief 尝试重新连接设备
   */
  void tryReconnect();

private: // 成员变量
  std::string file_name_; // 设备文件地址
  int fd_ = -1; // 设备文件描述符

  // 轴和按钮状态
  std::vector<int16_t> axis_state_;
  std::vector<uint8_t> button_state_;
  uint8_t axis_count_ = 0;
  uint8_t button_count_ = 0;

  // 轮询线程和相关变量
  std::thread timer_thread_;
  std::chrono::steady_clock::time_point last_received_time_;
  std::chrono::steady_clock::time_point last_reconnect_time_;
  std::atomic<bool> is_running_ = false;

};

} // namespace usbjs_driver

#endif // USB_JOYSTICK_MAIN_HPP