#ifndef RC_GIMBAL_MAIN_HPP
#define RC_GIMBAL_MAIN_HPP

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

namespace rc_gimbal
{

class RcGimbalMain
{
public: // constructor and destructor
  explicit RcGimbalMain(const char *file_name);
  ~RcGimbalMain();

public: // 公共接口
  int get_fd() const { return js_fd_; }
  int get_axis_state(int id) {
    if (id >= 0 && id < (int)axis_state_.size())
      return axis_state_[id];
    else
      return 0;
  };
  int get_button_state(int id) {
    if (id >= 0 && id < (int)button_state_.size())
      return button_state_[id];
    else
      return 0;
  };

private: // 打开设备
  int js_open(const char *file_name);

private: // 轮询线程
  void timerThread();
  void timerCallback();
  void tryReconnect();

private: // 成员变量
  std::string file_name_;
  int js_fd_ = -1;

  std::vector<int16_t> axis_state_;
  std::vector<uint8_t> button_state_;
  uint8_t axis_count_ = 0;
  uint8_t button_count_ = 0;

  std::thread timer_thread_;
  std::chrono::steady_clock::time_point last_received_time_;
  std::chrono::steady_clock::time_point last_reconnect_time_;
  std::atomic<bool> is_running_{false};

};

} // namespace rc_gimbal

#endif // RC_GIMBAL_MAIN_HPP