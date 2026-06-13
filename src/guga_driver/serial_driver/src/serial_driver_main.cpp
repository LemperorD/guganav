#include "serial_driver/serial_driver_main.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace serial_driver {

// ---- 终端颜色 ANSI 转义 ----
namespace termcolor {
static constexpr const char* reset{"\033[0m"};
static constexpr const char* red{"\033[31m"};
static constexpr const char* yellow{"\033[33m"};
static constexpr const char* green{"\033[32m"};
}  // namespace termcolor

// ==================== CRC8 ====================

uint8_t SerialDriverMain::crc8_calc(const uint8_t* p, size_t len) {
  uint8_t crc{CRC8_INIT};
  while (len--) {
    crc = CRC8_TABLE[crc ^ *p++];
  }
  return crc;
}

bool SerialDriverMain::isSupportedCommand(uint8_t cmd) {
  return (cmd == COMMAND_CODE_MOTION || cmd == COMMAND_CODE_REFEREE);
}

// ==================== 对外接收接口 ====================

uint8_t* SerialDriverMain::receiveDataFrame() {
  return frame_buffer_.data();
}

uint8_t* SerialDriverMain::receiveRefereeFrame() {
  return referee_frame_buffer_.data();
}

bool SerialDriverMain::hasNewRefereeFrame() const {
  return referee_frame_ready_.load();
}

void SerialDriverMain::clearRefereeFrameFlag() {
  referee_frame_ready_.store(false);
}

// ==================== 构造/析构 ====================

SerialDriverMain::SerialDriverMain(const std::string& serial_port,
                                   int baud_rate)
    : serial_port_(serial_port), baud_rate_(baud_rate) {
  // 打开或自动探测串口
  if (!serial_port_.empty()) {
    openSerialPort(serial_port_, baud_rate_);
  } else {
    std::cout << termcolor::yellow
              << "No serial port specified, auto-detecting..."
              << termcolor::reset << std::endl;
    std::string port = findSerialPort();
    if (!port.empty()) {
      openSerialPort(port, baud_rate_);
    } else {
      std::cerr << termcolor::red << "No serial port found."
                << termcolor::reset << std::endl;
    }
  }

  // 启动 ~1kHz 轮询线程
  running_ = true;
  timer_thread_ = std::thread(&SerialDriverMain::timerThread, this);

  // 初始化超时时间戳，避免启动时触发假超时
  last_received_time_ = std::chrono::steady_clock::now();
  last_reconnect_time_ = std::chrono::steady_clock::now();
}

SerialDriverMain::~SerialDriverMain() {
  running_ = false;
  if (timer_thread_.joinable()) {
    timer_thread_.join();
  }
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

// ==================== 串口打开与配置 ====================

void SerialDriverMain::openSerialPort(const std::string& port_name,
                                      int baud_rate) {
  // O_RDWR | O_NOCTTY | O_NDELAY: 读写模式，不成为控制终端，非阻塞打开
  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    std::cerr << termcolor::red
              << "Failed to open serial port: " << strerror(errno)
              << termcolor::reset << std::endl;
  } else {
    printf("\033[32mSerial port opened: %s\033[0m\n", port_name.c_str());
    configureSerialPort(baud_rate);
    printf("\033[32mSerial initialized: %s\033[0m\n", port_name.c_str());
  }
}

std::string SerialDriverMain::findSerialPort() {
  DIR* dir = opendir("/dev");
  if (!dir) {
    std::cerr << termcolor::red << "Failed to open /dev directory"
              << termcolor::reset << std::endl;
    return "";
  }

  // 按目录顺序返回第一个匹配的 ttyUSB 或 ttyACM 设备
  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strstr(entry->d_name, "ttyUSB") != nullptr
        || strstr(entry->d_name, "ttyACM") != nullptr) {
      closedir(dir);
      return std::string("/dev/") + entry->d_name;
    }
  }
  closedir(dir);
  return "";
}

void SerialDriverMain::configureSerialPort(int baud_rate) {
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << termcolor::red << "Failed to get serial attributes"
              << termcolor::reset << std::endl;
    close(fd_);
    fd_ = -1;
    return;
  }

  // 波特率映射
  speed_t speed;
  switch (baud_rate) {
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    case 230400:
      speed = B230400;
      break;
    default:
      std::cerr << termcolor::red << "Unsupported baud rate: " << baud_rate
                << termcolor::reset << std::endl;
      return;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1 原始模式
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;    // 无校验
  tty.c_cflag &= ~CSTOPB;    // 1 停止位
  tty.c_cflag &= ~CRTSCTS;   // 无硬件流控

  tty.c_lflag &= ~ICANON;    // 非规范模式（逐字节读取）
  tty.c_lflag &= ~ECHO;      // 关闭回显
  tty.c_lflag &= ~ISIG;      // 关闭信号字符

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 无软件流控
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;     // 原始输出

  // VMIN=0 VTIME=1: 非阻塞读取，最多等待 0.1 秒
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << termcolor::red << "Failed to set serial attributes"
              << termcolor::reset << std::endl;
    close(fd_);
    fd_ = -1;
    return;
  }

  // 清空收发缓冲区中的残留数据
  tcflush(fd_, TCIOFLUSH);
}

// ==================== 发送 ====================

void SerialDriverMain::sendDataFrame(const uint8_t* data, size_t len) {
  if (fd_ < 0) {
    std::cerr << termcolor::red << "Serial port not available"
              << termcolor::reset << std::endl;
    return;
  }

  // 组装帧：SOF(2) + CMD(1) + LEN(1) + PAYLOAD(len) + CRC8(1)
  const size_t frame_len = len + FRAME_MIN_SIZE;
  std::vector<uint8_t> frame(frame_len);

  frame[0] = FRAME_HEADER1;
  frame[1] = FRAME_HEADER2;
  frame[2] = COMMAND_CODE_MOTION;  // 上位机主动发送默认使用运动帧命令码
  frame[3] = static_cast<uint8_t>(len);
  std::memcpy(&frame[4], data, len);

  // CRC8 校验覆盖 SOF + CMD + LEN + PAYLOAD
  frame[frame_len - 1] = crc8_calc(frame.data(), 4 + len);

  const ssize_t written = write(fd_, frame.data(), frame_len);
  if (written != static_cast<ssize_t>(frame_len)) {
    std::cerr << termcolor::red << "TX failed " << written << "/" << frame_len
              << termcolor::reset << std::endl;
  }
}

// ==================== 接收与帧解析 ====================

void SerialDriverMain::processBuffer() {
  size_t frames_processed{};

  // 循环解析，直到缓冲区不满足最小帧长或达到处理上限
  while (buffer_index_ >= FRAME_MIN_SIZE
         && frames_processed < MAX_FRAMES_PER_LOOP) {
    // ---- 搜索帧头 0x42 0x52 ----
    size_t pos{};
    bool found{false};

    while (pos + 2 <= buffer_index_) {
      if (buffer_[pos] == FRAME_HEADER1 && buffer_[pos + 1] == FRAME_HEADER2) {
        found = true;
        break;
      }
      ++pos;
    }

    if (!found) {
      // 未找到帧头，丢弃全部缓冲数据
      buffer_index_ = 0;
      return;
    }

    // 丢弃帧头前的无效字节
    if (pos > 0) {
      std::memmove(buffer_.data(), buffer_.data() + pos, buffer_index_ - pos);
      buffer_index_ -= pos;
    }

    // 至少需要 SOF(2) + CMD(1) + LEN(1) 四个字节
    if (buffer_index_ < 4) {
      return;
    }

    // ---- 校验命令码 ----
    const uint8_t cmd = buffer_[2];
    if (!isSupportedCommand(cmd)) {
      std::cout << termcolor::yellow << "Unknown CMD=0x" << std::hex
                << static_cast<int>(cmd) << " (expected 0x"
                << static_cast<int>(COMMAND_CODE_MOTION) << " or 0x"
                << static_cast<int>(COMMAND_CODE_REFEREE) << ")" << std::dec
                << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    // ---- 确认帧长度 ----
    const uint8_t len = buffer_[3];
    const size_t frame_len = static_cast<size_t>(len) + FRAME_MIN_SIZE;

    if (frame_len > BUFFER_SIZE || frame_len > MAX_FRAME_LEN) {
      std::cout << termcolor::red << "Invalid frame length: " << frame_len
                << ", drop buffer" << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    // 缓冲区中数据不足完整帧，等待下次读取
    if (buffer_index_ < frame_len) {
      return;
    }

    // ---- CRC8 校验 ----
    const uint8_t calc = crc8_calc(buffer_.data(), 4 + len);
    if (calc != buffer_[frame_len - 1]) {
      std::cout << termcolor::yellow << "CRC8 failed: calc=0x" << std::hex
                << static_cast<int>(calc) << ", recv=0x"
                << static_cast<int>(buffer_[frame_len - 1])
                << ", len=" << std::dec << static_cast<int>(len)
                << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    // ---- 分发有效帧 ----
    processFrame(buffer_.data());
    ++frames_processed;

    // 消费已处理的帧数据，前移剩余缓冲区
    if (frame_len < buffer_index_) {
      std::memmove(buffer_.data(), buffer_.data() + frame_len,
                   buffer_index_ - frame_len);
      buffer_index_ -= frame_len;
    } else {
      buffer_index_ = 0;
    }
  }
}

void SerialDriverMain::processFrame(const uint8_t* data) {
  const uint8_t cmd = data[2];
  const uint8_t len = data[3];
  const uint8_t* pl = &data[4];  // payload 起始

  // 裁判系统帧 → referee_frame_buffer_
  if (cmd == COMMAND_CODE_REFEREE) {
    if (len == referee_frame_buffer_.size()) {
      std::memcpy(referee_frame_buffer_.data(), pl,
                  referee_frame_buffer_.size());
      referee_frame_ready_.store(true);
    } else {
      std::cout << termcolor::yellow
                << "Referee frame len mismatch: " << static_cast<int>(len)
                << " (expected " << referee_frame_buffer_.size() << ")"
                << termcolor::reset << std::endl;
    }
    return;
  }

  // 运动控制帧 → frame_buffer_
  if (cmd == COMMAND_CODE_MOTION) {
    if (len <= frame_buffer_.size()) {
      std::memcpy(frame_buffer_.data(), pl, len);
    } else {
      std::cout << termcolor::yellow
                << "Motion frame too long: " << static_cast<int>(len)
                << " (buffer " << frame_buffer_.size() << ")"
                << termcolor::reset << std::endl;
    }
    return;
  }

  std::cout << termcolor::yellow << "Unknown CMD=0x" << std::hex
            << static_cast<int>(cmd) << " LEN=" << std::dec
            << static_cast<int>(len) << " (ignored)" << termcolor::reset
            << std::endl;
}

// ==================== 轮询线程 ====================

void SerialDriverMain::timerThread() {
  while (running_) {
    const auto start = std::chrono::steady_clock::now();
    timerCallback();
    // 保证 1ms 固定周期
    std::this_thread::sleep_until(start + std::chrono::microseconds(1000));
  }
}

void SerialDriverMain::timerCallback() {
  // ---- 串口未打开，尝试重连 ----
  if (fd_ < 0) {
    if (std::chrono::steady_clock::now() - last_reconnect_time_
        > std::chrono::seconds(3)) {
      std::cerr << "Serial port not available, trying reconnect" << std::endl;
      tryReconnect();
    }
    return;
  }

  // ---- 3 秒无数据，触发重连 ----
  if (std::chrono::steady_clock::now() - last_received_time_
      > std::chrono::seconds(3)) {
    if (std::chrono::steady_clock::now() - last_reconnect_time_
        > std::chrono::seconds(3)) {
      std::cerr << "No data received, trying reconnect" << std::endl;
      tryReconnect();
    }
    return;
  }

  // ---- 缓冲区接近满，丢弃当前数据防止溢出 ----
  if (buffer_index_ >= BUFFER_SIZE - 64) {
    std::cout << termcolor::yellow << "Buffer near full, clearing"
              << termcolor::reset << std::endl;
    buffer_index_ = 0;
  }

  // ---- 从串口读数据 ----
  uint8_t temp[128];
  const ssize_t n = read(fd_, temp, sizeof(temp));
  if (n > 0) {
    last_received_time_ = std::chrono::steady_clock::now();

    // 环形缓冲溢出检查
    if (buffer_index_ + static_cast<size_t>(n) > BUFFER_SIZE) {
      std::cout << termcolor::red << "Buffer overflow, drop"
                << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    // 追加到环形缓冲并触发帧解析
    std::memcpy(buffer_.data() + buffer_index_, temp,
                static_cast<size_t>(n));
    buffer_index_ += static_cast<size_t>(n);
    processBuffer();
  }
}

// ==================== 重连机制 ====================

void SerialDriverMain::tryReconnect() {
  last_reconnect_time_ = std::chrono::steady_clock::now();

  // 关闭旧文件描述符
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }

  // 重置接收状态
  buffer_index_ = 0;
  referee_frame_ready_.store(false);

  // 重新打开串口
  if (!serial_port_.empty()) {
    openSerialPort(serial_port_, baud_rate_);
  } else {
    std::cout << termcolor::yellow
              << "No serial port specified, auto-detecting..."
              << termcolor::reset << std::endl;
    std::string port = findSerialPort();
    if (!port.empty()) {
      openSerialPort(port, baud_rate_);
    } else {
      std::cerr << termcolor::red << "waiting..." << termcolor::reset
                << std::endl;
    }
  }

  // 重连后重置时间戳，给新连接 3 秒宽限期
  last_reconnect_time_ = std::chrono::steady_clock::now();
  last_received_time_ = std::chrono::steady_clock::now();
}

// ==================== 浮点编解码 ====================

void SerialDriverMain::writeFloatLE(uint8_t* dst, float value) {
  uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(float));

  // 小端序：低位在前
  dst[0] = static_cast<uint8_t>(bits & 0xFFu);
  dst[1] = static_cast<uint8_t>((bits >> 8) & 0xFFu);
  dst[2] = static_cast<uint8_t>((bits >> 16) & 0xFFu);
  dst[3] = static_cast<uint8_t>((bits >> 24) & 0xFFu);
}

float SerialDriverMain::readFloatLE(const uint8_t* src) {
  // 小端序：低位在前
  const uint32_t bits = (static_cast<uint32_t>(src[0]))
                      | (static_cast<uint32_t>(src[1]) << 8)
                      | (static_cast<uint32_t>(src[2]) << 16)
                      | (static_cast<uint32_t>(src[3]) << 24);

  float value{};
  std::memcpy(&value, &bits, sizeof(float));
  return value;
}

}  // namespace serial_driver
