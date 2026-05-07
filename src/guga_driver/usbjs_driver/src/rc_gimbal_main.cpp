#include "rc_gimbal/rc_gimbal_main.hpp"

// #include <linux/joystick.h>

namespace rc_gimbal
{

RcGimbalMain::RcGimbalMain(const char *file_name)
  : file_name_(file_name)
{
  js_open(file_name_.c_str());
  ioctl(js_fd_, JSIOCGAXES, &axis_count_);
  ioctl(js_fd_, JSIOCGBUTTONS, &button_count_);
  axis_state_.resize(axis_count_);
  button_state_.resize(button_count_);

  // Get Name
  char name_c_str[1024];
  if (ioctl(js_fd_, JSIOCGNAME(sizeof(name_c_str)), name_c_str) < 0)
  {
    std::ostringstream str;
    str << file_name_.c_str() << ": " << strerror(errno);
    throw std::runtime_error(str.str());
  }

  is_running_ = true;
  timer_thread_ = std::thread(&RcGimbalMain::timerThread, this);

  last_received_time_ = std::chrono::steady_clock::now();
  last_reconnect_time_ = std::chrono::steady_clock::now();

  std::cout << "\033[32m"
            << "RcGimbalMain constructor called with file name: " << file_name_.c_str() 
            << "\033[0m" << std::endl;
}

RcGimbalMain::~RcGimbalMain()
{
  is_running_ = false;
  if (timer_thread_.joinable()) timer_thread_.join();

  if (js_fd_ >= 0) {
    if (close(js_fd_) < 0) {
      std::cerr << "\033[31m" << "Failed to close joystick device, error: " << strerror(errno) << "\033[0m" << std::endl;
    }
    js_fd_ = -1;
    std::cout << "\033[32m" << file_name_.c_str() << " closed" << "\033[0m" << std::endl;
  }

  std::cout << "\033[32m" << "RcGimbalMain destructor called" << "\033[0m" << std::endl;
}

int RcGimbalMain::js_open(const char *file_name)
{
  // Open joystick device in non-blocking mode so reads return immediately when
  // there is no data instead of blocking the timer thread.
  js_fd_ = open(file_name, O_RDONLY | O_NONBLOCK);
  if (js_fd_ < 0) {
    std::cerr << "\033[31m" << "Failed to open joystick device: " << file_name << ", error: " << strerror(errno) << "\033[0m" << std::endl;
    return -1;
  }
  return js_fd_;
}

void RcGimbalMain::timerThread()
{
  while (is_running_) {
    timerCallback();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms
  }
}

void RcGimbalMain::timerCallback() {
  if (js_fd_ < 0) {
    if (std::chrono::steady_clock::now() - last_reconnect_time_ > std::chrono::seconds(10)) {
      std::cerr << "Joystick not available, trying reconnect" << std::endl;
      tryReconnect();
    }
    return;
  }

  struct js_event event;
  ssize_t len = read(js_fd_, &event, sizeof(event));

  if (len < 0) {
    // Non-blocking read: if no data available, just return and wait.
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return;
    }
    std::ostringstream str;
    str << file_name_.c_str() << ": " << strerror(errno);
    throw std::runtime_error(str.str());
  } else if (len == sizeof(event)) {
    // 成功读取到事件，更新接收时间并更新状态
    last_received_time_ = std::chrono::steady_clock::now();
    uint8_t etype = event.type & ~JS_EVENT_INIT; // mask init flag
    if (etype & JS_EVENT_AXIS) {
      if (event.number >= 0 && event.number < (int)axis_state_.size())
        axis_state_[event.number] = event.value;
    } else if (etype & JS_EVENT_BUTTON) {
      if (event.number >= 0 && event.number < (int)button_state_.size())
        button_state_[event.number] = event.value;
    }
  } else {
    std::ostringstream str;
    str << "RcGimbalMain::timerCallback(): unknown read length " << len;
    throw std::runtime_error(str.str());
  }
}

void RcGimbalMain::tryReconnect() {
  last_reconnect_time_ = std::chrono::steady_clock::now();
  if (js_fd_ >= 0) {
    close(js_fd_);
    js_fd_ = -1;
  }

  js_open(file_name_.c_str());

  last_reconnect_time_ = std::chrono::steady_clock::now();
  last_received_time_ = std::chrono::steady_clock::now();
}


} // namespace rc_gimbal