#ifndef SERIAL_DRIVER_MAIN_HPP
#define SERIAL_DRIVER_MAIN_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace serial_driver {

// ================= BR 串口协议常量 =================
// SOF = 'B' 'R'  (0x42 0x52) (Beihang Robotics)
static constexpr uint8_t FRAME_HEADER1{0x42};
static constexpr uint8_t FRAME_HEADER2{0x52};

// 自定义命令码：下位机与上位机统一
// 0xCD: 运动控制帧  0xD1: 裁判系统帧
static constexpr uint8_t COMMAND_CODE_MOTION{0xCD};
static constexpr uint8_t COMMAND_CODE_REFEREE{0xD1};

// 缓冲与帧尺寸（最小帧 = 2B SOF + 1B CMD + 1B LEN + 1B CRC8 = 5）
static constexpr size_t BUFFER_SIZE{256};
static constexpr size_t FRAME_MIN_SIZE{5};
static constexpr size_t MAX_FRAME_LEN{128};
// 避免长时间占用：单次 timer 回调最多处理10帧
static constexpr size_t MAX_FRAMES_PER_LOOP{10};

// CRC8 参数：RoboMaster 电控常用表驱动, poly=0x31, init=0xFF
static constexpr uint8_t CRC8_INIT{0xFF};
static const uint8_t CRC8_TABLE[256]{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35};

/**
 * @brief 串口通信底层类，负责与下位机(MCU)的物理层串口交互。
 *
 * 职责：串口打开/配置/关闭、数据帧收发、CRC8 校验、帧缓冲管理、断线自动重连。
 * 不依赖任何 ROS2 类型，可独立复用。
 *
 * 帧协议（BR 协议）：
 *   - 帧头: 0x42 0x52 ("BR" = Beihang Robotics)
 *   - 格式: SOF(2B) | CMD(1B) | LEN(1B) | PAYLOAD(nB) | CRC8(1B)
 *   - 命令码: 0xCD = 运动控制帧, 0xD1 = 裁判系统帧
 *   - CRC8: poly=0x31, init=0xFF, 查表驱动
 */
class SerialDriverMain {
 public:
  /**
   * @brief 构造函数，打开串口并启动内部轮询线程（~1kHz）。
   * @param serial_port 串口设备路径（如 "/dev/ttyACM0"），为空时自动探测 ttyUSB/ttyACM。
   * @param baud_rate 波特率，支持 9600/19200/38400/57600/115200/230400。
   */
  explicit SerialDriverMain(const std::string& serial_port = "",
                            int baud_rate = 115200);

  /** @brief 析构函数，停止轮询线程并关闭串口。 */
  ~SerialDriverMain();

  // ---- MCU → PC 接收接口 ----

  /**
   * @brief 获取最新运动控制帧的 payload 指针（26 字节缓冲区）。
   * @return 指向 frame_buffer_ 的指针，数据在每次收到有效运动帧时更新。
   * @note 调用者不应持有此指针超过下一次 timerCallback。
   */
  [[nodiscard]] uint8_t* receiveDataFrame();

  /**
   * @brief 获取最新裁判系统帧的 payload 指针（13 字节缓冲区）。
   * @return 指向 referee_frame_buffer_ 的指针。
   */
  [[nodiscard]] uint8_t* receiveRefereeFrame();

  /**
   * @brief 查询是否有新的裁判系统帧到达。
   * @return true 表示有新帧等待消费。
   */
  [[nodiscard]] bool hasNewRefereeFrame() const;

  /** @brief 清除裁判系统帧的"新帧到达"标志，消费后调用。 */
  void clearRefereeFrameFlag();

  // ---- PC → MCU 发送接口 ----

  /**
   * @brief 将 payload 封装为 BR 协议帧并通过串口发送。
   * @param data 指向 payload 数据的指针。
   * @param len payload 字节数。
   *
   * 自动添加帧头(0x42 0x52)、命令码(0xCD)、长度字段、CRC8 尾部。
   */
  void sendDataFrame(const uint8_t* data, size_t len);

  // ---- 浮点编解码（小端序） ----

  /**
   * @brief 将 float 按小端序写入 4 字节缓冲区。
   * @param dst 目标缓冲区（至少 4 字节）。
   * @param value 要写入的浮点数。
   */
  static void writeFloatLE(uint8_t* dst, float value);

  /**
   * @brief 从 4 字节小端序缓冲区读取 float。
   * @param src 源缓冲区（至少 4 字节，小端序）。
   * @return 解码后的浮点数。
   */
  [[nodiscard]] static float readFloatLE(const uint8_t* src);

 private:
  // ---- 轮询线程 ----

  /** @brief 轮询线程主循环，以 ~1ms 周期调用 timerCallback。 */
  void timerThread();

  /**
   * @brief 单次轮询回调：从串口读取数据填入环形缓冲，触发帧解析。
   *
   * 同时负责超时检测（3 秒无数据则重连）和缓冲区溢出保护。
   */
  void timerCallback();

  // ---- 帧缓冲解析 ----

  /**
   * @brief 从接收环形缓冲区中搜索并解析完整的 BR 协议帧。
   *
   * 搜索帧头 0x42 0x52 → 校验命令码 → 确认长度 → CRC8 校验 → 分发有效帧。
   * 单次调用最多处理 MAX_FRAMES_PER_LOOP 帧以避免长时间阻塞。
   */
  void processBuffer();

  /**
   * @brief 将已校验的有效帧按命令码分发到对应的 payload 缓冲区。
   * @param data 指向完整帧起始位置（含 SOF）的指针。
   *
   * - COMMAND_CODE_REFEREE → referee_frame_buffer_，同时置 referee_frame_ready_ 标志
   * - COMMAND_CODE_MOTION → frame_buffer_
   */
  void processFrame(const uint8_t* data);

  // ---- 串口操作 ----

  /**
   * @brief 以非阻塞模式打开串口设备并配置波特率。
   * @param port_name 设备路径，如 "/dev/ttyACM0"。
   * @param baud_rate 波特率。
   */
  void openSerialPort(const std::string& port_name, int baud_rate);

  /**
   * @brief 自动探测 /dev/ 下第一个 ttyUSB* 或 ttyACM* 设备。
   * @return 发现的设备路径，没有则返回空串。
   */
  [[nodiscard]] std::string findSerialPort();

  /**
   * @brief 配置串口为 8N1 原始模式，无流控，VTIME=1。
   * @param baud_rate 波特率。
   */
  void configureSerialPort(int baud_rate);

  // ---- 断线重连 ----

  /** @brief 关闭当前串口并尝试重新打开，重置缓冲区状态。 */
  void tryReconnect();

  // ---- CRC8 ----

  /**
   * @brief 计算 BR 协议 CRC8（poly=0x31, init=0xFF, 查表法）。
   * @param p 数据起始指针。
   * @param len 参与校验的数据长度（不含 CRC 字节自身）。
   * @return CRC8 校验值。
   */
  [[nodiscard]] static uint8_t crc8_calc(const uint8_t* p, size_t len);

  /**
   * @brief 判断命令码是否为已支持的指令类型。
   * @param cmd 命令码。
   * @return true 表示 cmd 为 COMMAND_CODE_MOTION 或 COMMAND_CODE_REFEREE。
   */
  [[nodiscard]] static bool isSupportedCommand(uint8_t cmd);

 private:
  // 文件描述符，-1 表示未打开
  int fd_{-1};
  bool running_{false};

  // 串口参数
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{115200};

  // 接收环形缓冲
  std::array<uint8_t, BUFFER_SIZE> buffer_{};
  size_t buffer_index_{};

  // 运动控制帧 payload 缓冲（25 字节有效载荷 + 1 字节余量）
  std::array<uint8_t, 26> frame_buffer_{};

  // 裁判系统帧 payload 缓冲（13 字节）
  std::array<uint8_t, 13> referee_frame_buffer_{};
  std::atomic_bool referee_frame_ready_{false};

  // 轮询线程
  std::thread timer_thread_;

  // 超时/重连计时
  std::chrono::steady_clock::time_point last_received_time_;
  std::chrono::steady_clock::time_point last_reconnect_time_;
};

}  // namespace serial_driver

#endif  // SERIAL_DRIVER_MAIN_HPP
