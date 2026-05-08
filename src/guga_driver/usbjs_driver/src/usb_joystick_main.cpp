#include "usbjs_driver/usb_joystick_main.hpp"

namespace usbjs_driver
{

UsbJoystick::UsbJoystick(const char *file_name)
  : file_name_(file_name)
{
  js_open(file_name_.c_str());
  ioctl(fd_, JSIOCGAXES, &axis_count_);
  ioctl(fd_, JSIOCGBUTTONS, &button_count_);
  axis_state_.resize(axis_count_);
  button_state_.resize(button_count_);

  char name_c_str[1024];
  if (ioctl(fd_, JSIOCGNAME(sizeof(name_c_str)), name_c_str) < 0)
  {
    std::ostringstream str;
    str << file_name_.c_str() << ": " << strerror(errno);
    throw std::runtime_error(str.str());
  }
  std::cout << "\033[32m"
          << "UsbJoystick name: " << name_c_str
          << "\033[0m" << std::endl;

  is_running_ = true;
  timer_thread_ = std::thread(&UsbJoystick::timerThread, this);
  last_received_time_ = std::chrono::steady_clock::now();
  last_reconnect_time_ = std::chrono::steady_clock::now();

  std::cout << "\033[32m"
            << "UsbJoystick constructor called with file name: " << file_name_.c_str() 
            << "\033[0m" << std::endl;
}

UsbJoystick::~UsbJoystick()
{
  is_running_ = false;
  if (timer_thread_.joinable()) timer_thread_.join();

  if (fd_ >= 0) {
    if (close(fd_) < 0) {
      std::cerr << "\033[31m"
                << "Failed to close joystick device, error: " << strerror(errno)
                << "\033[0m" << std::endl;
    }
    fd_ = -1;
    std::cout << "\033[32m"
              << file_name_.c_str() << " closed"
              << "\033[0m" << std::endl;
  }

  std::cout << "\033[32m"
            << "UsbJoystick destructor called"
            << "\033[0m" << std::endl;
}

int UsbJoystick::js_open(const char *file_name)
{
  fd_ = open(file_name, O_RDONLY | O_NONBLOCK); // 以只读和非阻塞模式打开设备
  if (fd_ < 0) {
    std::cerr << "\033[31m"
              << "Failed to open joystick device: " << file_name
              << ", error: " << strerror(errno)
              << "\033[0m" << std::endl;
    return -1;
  }
  return fd_;
}

void UsbJoystick::timerThread()
{
  while (is_running_) {
    timerCallback();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms
  }
}

void UsbJoystick::timerCallback() {
  if (fd_ < 0) { // 设备未打开，尝试重新连接
    if (std::chrono::steady_clock::now() - last_reconnect_time_ > std::chrono::seconds(10)) {
      std::cerr << "Joystick not available, trying reconnect" << std::endl;
      tryReconnect();
    }
    return;
  }

  struct js_event event;
  ssize_t len = read(fd_, &event, sizeof(event));

  // 处理读取结果
  if (len < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return;
    std::ostringstream str;
    str << file_name_.c_str() << ": " << strerror(errno);
    throw std::runtime_error(str.str());
  }
  // 成功读取到事件，更新接收时间并更新状态
  else if (len == sizeof(event)) {
    last_received_time_ = std::chrono::steady_clock::now();
    uint8_t etype = event.type & ~JS_EVENT_INIT; // 过滤掉初始化事件标志
    // 根据事件类型更新轴或按钮状态
    if (etype & JS_EVENT_AXIS) {
      // 更新轴状态
      if (event.number >= 0 && event.number < (int)axis_state_.size())
        axis_state_[event.number] = event.value;
    } else if (etype & JS_EVENT_BUTTON) {
      // 更新按钮状态
      if (event.number >= 0 && event.number < (int)button_state_.size())
        button_state_[event.number] = event.value;
    }
  }
  // 读取的事件长度异常
  else {
    std::ostringstream str;
    str << "RcGimbalMain::timerCallback(): unknown read length " << len;
    throw std::runtime_error(str.str());
  }
}

void UsbJoystick::tryReconnect() {
  last_reconnect_time_ = std::chrono::steady_clock::now();

  // 关闭设备，重启
  if (fd_ >= 0) {
    close(fd_); fd_ = -1;
  }
  js_open(file_name_.c_str());

  last_reconnect_time_ = std::chrono::steady_clock::now();
  last_received_time_ = std::chrono::steady_clock::now();
}

} // namespace usbjs_driver