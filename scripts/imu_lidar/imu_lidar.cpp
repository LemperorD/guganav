// MID-360 板载 IMU 读取与六面采集工具（独立程序，不依赖 ROS）。
//
// 数据来源是 Livox SDK2 的 IMU 回调，走网口，不经过 ROS 话题，也不经过串口。
// 雷达与主机的 IP、端口由配置文件决定，实车配置在
// src/guga_bringup/config/reality/mid360_user_config.json。
//
// SDK 回调在它自己的线程里执行，采集缓存用互斥量保护。
//
// 六面采集输出的 CSV 与 scripts/imu_calib 的输入一致：
//   t,ax,ay,az,wx,wy,wz,face
// 加速度单位 m/s^2，角速度单位 rad/s。

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr double kGravity = 9.80665;

volatile std::sig_atomic_t g_running = 1;

void handle_sigint(int) { g_running = 0; }

double now_seconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

struct ImuSample {
  double t = 0.0;
  double acc[3] = {0.0, 0.0, 0.0};
  double gyro[3] = {0.0, 0.0, 0.0};
};

// ===================== 采集缓存 =====================

class Collector {
 public:
  void push(const ImuSample& sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++total_;
    recent_.push_back(sample);
    while (!recent_.empty() && sample.t - recent_.front().t > 2.0) {
      recent_.pop_front();
    }
    if (collecting_) {
      rows_.emplace_back(face_, sample);
    }
  }

  unsigned long long total() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_;
  }

  size_t recent_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return recent_.size();
  }

  void clear_recent() {
    std::lock_guard<std::mutex> lock(mutex_);
    recent_.clear();
  }

  // 最近 window 秒内角速度模长的标准差，用来判断是否静止。
  double gyro_std(double window) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (recent_.empty()) {
      return 1e9;
    }
    const double latest = recent_.back().t;
    double sum = 0.0;
    double square_sum = 0.0;
    size_t count = 0;
    for (auto it = recent_.rbegin(); it != recent_.rend(); ++it) {
      if (latest - it->t > window) {
        break;
      }
      const double magnitude = std::sqrt(
          it->gyro[0] * it->gyro[0] + it->gyro[1] * it->gyro[1]
          + it->gyro[2] * it->gyro[2]);
      sum += magnitude;
      square_sum += magnitude * magnitude;
      ++count;
    }
    if (count < 2) {
      return 1e9;
    }
    const double mean = sum / static_cast<double>(count);
    return std::sqrt(std::max(
        0.0, square_sum / static_cast<double>(count) - mean * mean));
  }

  void begin_collect(const std::string& face) {
    std::lock_guard<std::mutex> lock(mutex_);
    face_ = face;
    collecting_ = true;
  }

  void end_collect() {
    std::lock_guard<std::mutex> lock(mutex_);
    collecting_ = false;
  }

  size_t row_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rows_.size();
  }

  std::vector<std::pair<std::string, ImuSample>> rows_since(
      size_t begin) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (begin >= rows_.size()) {
      return {};
    }
    return {rows_.begin() + static_cast<std::ptrdiff_t>(begin), rows_.end()};
  }

  // 最近 window 秒内加速度的均值，用来在采样前预览当前朝向。
  bool mean_acc(double window, double out[3], size_t& count) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (recent_.empty()) {
      return false;
    }
    const double latest_time = recent_.back().t;
    double sum[3] = {0.0, 0.0, 0.0};
    size_t n = 0;
    for (auto it = recent_.rbegin(); it != recent_.rend(); ++it) {
      if (latest_time - it->t > window) {
        break;
      }
      for (int axis = 0; axis < 3; ++axis) {
        sum[axis] += it->acc[axis];
      }
      ++n;
    }
    if (n == 0) {
      return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
      out[axis] = sum[axis] / static_cast<double>(n);
    }
    count = n;
    return true;
  }

  bool latest(ImuSample& sample) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (recent_.empty()) {
      return false;
    }
    sample = recent_.back();
    return true;
  }

 private:
  mutable std::mutex mutex_;
  std::deque<ImuSample> recent_;
  std::vector<std::pair<std::string, ImuSample>> rows_;
  std::string face_;
  bool collecting_ = false;
  unsigned long long total_ = 0;
};

// ===================== 模拟数据源 =====================

// 没有硬件时用已知参数生成样本，用来验证六面流程与 CSV 输出。
class Simulator {
 public:
  explicit Simulator(Collector& collector)
      : collector_(collector), random_(7) {}

  ~Simulator() { stop(); }

  void start() {
    running_ = true;
    thread_ = std::thread([this] { run(); });
  }

  void stop() {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void set_face(const std::string& face) {
    std::lock_guard<std::mutex> lock(face_mutex_);
    face_ = face;
  }

 private:
  void run() {
    const auto period = std::chrono::microseconds(5000);  // 200 Hz
    std::normal_distribution<double> acc_noise(0.0, 0.01);
    std::normal_distribution<double> gyro_noise(0.0, 0.002);
    auto next = std::chrono::steady_clock::now();

    while (running_ && g_running != 0) {
      std::string face;
      {
        std::lock_guard<std::mutex> lock(face_mutex_);
        face = face_.empty() ? "+z" : face_;
      }

      const int axis = face[1] - 'x';
      const double sign = face[0] == '+' ? 1.0 : -1.0;
      double true_acc[3] = {0.0, 0.0, 0.0};
      true_acc[axis] = sign * kGravity;

      ImuSample sample;
      sample.t = now_seconds();
      for (int row = 0; row < 3; ++row) {
        double value = bias_[row] + acc_noise(random_);
        for (int column = 0; column < 3; ++column) {
          value += matrix_[row][column] * true_acc[column];
        }
        sample.acc[row] = value;
      }
      for (int i = 0; i < 3; ++i) {
        sample.gyro[i] = gyro_bias_[i] + gyro_noise(random_);
      }
      collector_.push(sample);

      next += period;
      const auto now = std::chrono::steady_clock::now();
      if (next < now - period) {
        next = now;
      }
      std::this_thread::sleep_until(next);
    }
  }

  Collector& collector_;
  std::mt19937 random_;
  std::array<std::array<double, 3>, 3> matrix_{{{1.021, 0.006, -0.004},
                                                {0.003, 0.985, 0.005},
                                                {-0.005, 0.004, 1.013}}};
  std::array<double, 3> bias_{{0.15, -0.21, 0.09}};
  std::array<double, 3> gyro_bias_{{0.010, -0.020, 0.005}};
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex face_mutex_;
  std::string face_ = "+z";
};

// ===================== Livox 回调 =====================

struct AppContext {
  Collector* collector = nullptr;
  std::atomic<bool> got_first_sample{false};
};

void on_work_mode(livox_status status, uint32_t handle,
                   LivoxLidarAsyncControlResponse* /*response*/,
                   void* /*client_data*/) {
  if (status != kLivoxLidarStatusSuccess) {
    std::cerr << "切换雷达工作模式失败，handle " << handle << "，状态码 "
              << static_cast<int>(status) << "\n";
  }
}

void on_enable(livox_status status, uint32_t handle,
               LivoxLidarAsyncControlResponse* /*response*/,
               void* /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "雷达 " << handle << " 已开启 IMU 数据\n";
  } else {
    std::cerr << "开启 IMU 数据失败，handle " << handle << "，状态码 "
              << static_cast<int>(status) << "\n";
  }
}

void on_imu(uint32_t /*handle*/, uint8_t /*dev_type*/,
            LivoxLidarEthernetPacket* data, void* client_data) {
  auto* context = static_cast<AppContext*>(client_data);
  if (context == nullptr || context->collector == nullptr || data == nullptr) {
    return;
  }
  if (data->data_type != kLivoxLidarImuData) {
    return;
  }

  const auto* raw = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
  ImuSample sample;
  sample.t = now_seconds();
  sample.acc[0] = raw->acc_x * kGravity;
  sample.acc[1] = raw->acc_y * kGravity;
  sample.acc[2] = raw->acc_z * kGravity;
  sample.gyro[0] = raw->gyro_x;
  sample.gyro[1] = raw->gyro_y;
  sample.gyro[2] = raw->gyro_z;
  context->collector->push(sample);
  context->got_first_sample.store(true);
}

void on_info_change(uint32_t handle, const LivoxLidarInfo* info,
                    void* client_data) {
  std::cout << "检测到雷达";
  if (info != nullptr) {
    std::cout << "，序列号 " << info->sn << "，IP " << info->lidar_ip;
  }
  std::cout << "，handle " << handle << "\n";

  SetLivoxLidarImuDataCallback(on_imu, client_data);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, on_work_mode, client_data);
  EnableLivoxLidarImuData(handle, on_enable, client_data);
}

// ===================== 参数与流程 =====================

struct Options {
  std::string config;
  std::string mode = "monitor";  // monitor / six-face
  std::string out = "imu_six_face.csv";
  double duration = 5.0;
  double still_timeout = 10.0;
  double still_std = 0.02;
  bool simulate = false;
};

const std::array<std::string, 6> kFaces = {"+x", "-x", "+y", "-y", "+z", "-z"};

std::string face_hint(const std::string& face) {
  const int axis = face[1] - 'x';
  const char* names[3] = {"X", "Y", "Z"};
  const bool up = face[0] == '+';
  return std::string("让传感器的 ") + names[axis] + " 轴竖直"
         + (up ? "朝上" : "朝下") + "，即 " + face + " 面朝"
         + (up ? "上" : "下");
}

std::string default_config_path(const char* argv0) {
  std::error_code ec;
  const auto exe = std::filesystem::absolute(argv0, ec);
  if (ec) {
    return "src/guga_bringup/config/reality/mid360_user_config.json";
  }
  const auto repo = exe.parent_path().parent_path().parent_path();
  return (repo / "src" / "guga_bringup" / "config" / "reality"
          / "mid360_user_config.json")
      .string();
}

bool wait_for_data(const Collector& collector, double timeout) {
  const double deadline = now_seconds() + timeout;
  while (now_seconds() < deadline && g_running != 0) {
    if (collector.total() > 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return collector.total() > 0;
}

bool wait_still(const Collector& collector, const Options& options) {
  const double deadline = now_seconds() + options.still_timeout;
  double last_report = 0.0;
  while (now_seconds() < deadline && g_running != 0) {
    if (collector.recent_size() >= 20
        && collector.gyro_std(0.5) <= options.still_std) {
      return true;
    }
    const double now = now_seconds();
    if (now - last_report > 0.5) {
      last_report = now;
      ImuSample sample;
      if (collector.latest(sample)) {
        std::cout << "  角速度模长 " << std::fixed << std::setprecision(3)
                  << std::sqrt(sample.gyro[0] * sample.gyro[0]
                               + sample.gyro[1] * sample.gyro[1]
                               + sample.gyro[2] * sample.gyro[2])
                  << " rad/s，等待静止..." << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

int collect_six_face(Collector& collector, Simulator* simulator,
                     const Options& options) {
  std::cout << "六面采集开始。每个朝向 " << options.duration
            << " s，按回车开始该朝向的采样。\n";

  for (const auto& face : kFaces) {
    if (g_running == 0) {
      return 1;
    }
    std::cout << "\n[" << face << "] " << face_hint(face) << "\n";

    const int expected_axis = face[1] - 'x';
    const bool expected_up = face[0] == '+';
    while (true) {
      std::cout << "  放稳并确认朝向正确后按回车开始采样: " << std::flush;
      std::string line;
      const bool interactive = static_cast<bool>(std::getline(std::cin, line));
      if (!interactive) {
        std::cout << "(标准输入已关闭，直接开始采样)\n";
      }

      if (simulator != nullptr) {
        simulator->set_face(face);
      }

      // 预览 0.8 s，确认主轴与提示一致，方向放反时让操作者重来。
      collector.clear_recent();
      std::this_thread::sleep_for(std::chrono::milliseconds(800));
      double mean[3] = {0.0, 0.0, 0.0};
      size_t count = 0;
      if (!collector.mean_acc(0.5, mean, count)) {
        std::cout << "  还没有收到数据，稍后重试\n";
        continue;
      }

      int dominant = 0;
      for (int axis = 1; axis < 3; ++axis) {
        if (std::abs(mean[axis]) > std::abs(mean[dominant])) {
          dominant = axis;
        }
      }
      std::cout << "  当前读数 [" << std::fixed << std::setprecision(3)
                << mean[0] << " " << mean[1] << " " << mean[2]
                << "] m/s^2，主轴 " << "XYZ"[dominant] << "\n";

      const bool axis_ok = dominant == expected_axis;
      const bool sign_ok =
          expected_up ? mean[dominant] > 0.0 : mean[dominant] < 0.0;
      if (axis_ok && sign_ok) {
        break;
      }

      std::cout << "  主轴与提示的 " << "XYZ"[expected_axis] << " 轴"
                << (expected_up ? "朝上" : "朝下") << " 不一致。";
      if (!interactive) {
        break;
      }
      std::cout << "检查朝向：按回车重试，输入 c 继续\n";
      std::string choice;
      if (!std::getline(std::cin, choice)) {
        break;
      }
      if (choice == "c" || choice == "C") {
        break;
      }
    }

    collector.clear_recent();
    if (!wait_still(collector, options)) {
      std::cout << "  注意: 等待静止超时，仍按当前数据采样，"
                   "请确认传感器没有晃动\n";
    }

    collector.clear_recent();
    const size_t begin = collector.row_count();
    collector.begin_collect(face);
    const double deadline = now_seconds() + options.duration;
    while (now_seconds() < deadline && g_running != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    collector.end_collect();

    const auto rows = collector.rows_since(begin);
    if (rows.size() < 20) {
      std::cerr << "  " << face << " 面只采到 " << rows.size()
                << " 个样本，检查雷达连接与配置\n";
      return 1;
    }

    double mean[3] = {0.0, 0.0, 0.0};
    for (const auto& row : rows) {
      for (int axis = 0; axis < 3; ++axis) {
        mean[axis] += row.second.acc[axis];
      }
    }
    for (double& value : mean) {
      value /= static_cast<double>(rows.size());
    }
    std::cout << "  采样 " << rows.size() << " 个，加速度均值 [" << std::fixed
              << std::setprecision(3) << mean[0] << " " << mean[1] << " "
              << mean[2] << "] m/s^2\n";
  }

  const auto rows = collector.rows_since(0);
  if (rows.empty()) {
    std::cerr << "没有采集到数据\n";
    return 1;
  }

  std::ofstream out(options.out);
  if (!out) {
    std::cerr << "无法写入 " << options.out << "\n";
    return 1;
  }
  out << "t,ax,ay,az,wx,wy,wz,face\n";
  const double start = rows.front().second.t;
  out << std::fixed << std::setprecision(6);
  for (const auto& row : rows) {
    const ImuSample& sample = row.second;
    out << (sample.t - start) << "," << sample.acc[0] << "," << sample.acc[1]
        << "," << sample.acc[2] << "," << sample.gyro[0] << ","
        << sample.gyro[1] << "," << sample.gyro[2] << "," << row.first << "\n";
  }
  out.close();

  std::cout << "\n六个朝向共 " << rows.size() << " 个样本，已写入 "
            << options.out << "\n";
  std::cout << "下一步: python3 scripts/imu_calib/imu_calib.py solve --csv "
            << options.out << "\n";
  return 0;
}

int monitor(Collector& collector) {
  std::cout << "显示 IMU 样本，按 Ctrl+C 退出\n";
  double last_print = 0.0;
  unsigned long long last_total = 0;
  double last_change = now_seconds();

  while (g_running != 0) {
    const double now = now_seconds();
    ImuSample sample;
    if (collector.latest(sample) && now - last_print > 0.2) {
      last_print = now;
      std::cout << std::fixed << std::setprecision(3) << "acc ["
                << sample.acc[0] << " " << sample.acc[1] << " "
                << sample.acc[2] << "]  gyro [" << sample.gyro[0] << " "
                << sample.gyro[1] << " " << sample.gyro[2] << "]" << std::endl;
    }
    const unsigned long long total = collector.total();
    if (total != last_total) {
      last_total = total;
      last_change = now;
    } else if (now - last_change > 5.0) {
      std::cerr << "5 s 内没有收到新样本，检查网线、雷达电源与配置文件 IP\n";
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return 0;
}

bool start_livox(AppContext& context, const std::string& config_path) {
  if (!std::filesystem::exists(config_path)) {
    std::cerr << "找不到配置文件: " << config_path << "\n";
    return false;
  }

  DisableLivoxSdkConsoleLogger();
  if (!LivoxLidarSdkInit(config_path.c_str())) {
    std::cerr << "Livox SDK 初始化失败。检查配置文件里的主机 IP 是否与网卡"
                 "地址一致，以及 56100 到 56501 端口是否被占用。\n";
    return false;
  }
  SetLivoxLidarInfoChangeCallback(on_info_change, &context);
  return true;
}

void print_usage() {
  std::cout
      << "用法: imu_lidar [选项]\n"
      << "  --config PATH    MID-360 网络配置文件，默认使用仓库内的实车配置\n"
      << "  --six-face       交互式六面采集\n"
      << "  --out PATH       六面采集输出 CSV，默认 imu_six_face.csv\n"
      << "  --duration SEC   每个朝向的采集时长，默认 5\n"
      << "  --still-timeout SEC 等待静止的最长时间，默认 10\n"
      << "  --still-gyro-std R  判定静止的角速度标准差，默认 0.02\n"
      << "  --simulate       用合成数据跑完整流程，不需要雷达\n"
      << "  --help           显示本说明\n";
}

std::optional<Options> parse_options(int argc, char** argv) {
  Options options;
  options.config = default_config_path(argv[0]);

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "缺少参数: " << name << "\n";
        std::exit(2);
      }
      return argv[++i];
    };

    if (arg == "--config") {
      options.config = next("--config");
    } else if (arg == "--six-face") {
      options.mode = "six-face";
    } else if (arg == "--out") {
      options.out = next("--out");
    } else if (arg == "--duration") {
      options.duration = std::stod(next("--duration"));
    } else if (arg == "--still-timeout") {
      options.still_timeout = std::stod(next("--still-timeout"));
    } else if (arg == "--still-gyro-std") {
      options.still_std = std::stod(next("--still-gyro-std"));
    } else if (arg == "--simulate") {
      options.simulate = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      std::exit(0);
    } else {
      std::cerr << "未知参数: " << arg << "\n";
      print_usage();
      return std::nullopt;
    }
  }
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, handle_sigint);

  const auto parsed = parse_options(argc, argv);
  if (!parsed) {
    return 2;
  }
  const Options options = *parsed;

  Collector collector;
  Simulator simulator(collector);

  AppContext context;
  context.collector = &collector;

  bool livox_started = false;
  if (options.simulate) {
    std::cout << "使用模拟数据源，不连接雷达\n";
    simulator.start();
  } else {
    std::cout << "配置文件: " << options.config << "\n";
    if (!start_livox(context, options.config)) {
      return 1;
    }
    livox_started = true;
    std::cout << "等待雷达连接与第一帧 IMU 数据...\n";
  }

  if (!wait_for_data(collector, 10.0)) {
    std::cerr << "10 s 内没有收到 IMU 数据。\n";
    std::cerr << "检查: 雷达上电、网线连接、电脑网口 IP 是否为配置里的主机 IP，"
                 "以及雷达 IP 是否为配置里的雷达 IP。\n";
    if (livox_started) {
      LivoxLidarSdkUninit();
    }
    return 1;
  }
  std::cout << "已收到 IMU 数据\n";

  int result = 0;
  if (options.mode == "six-face") {
    result = collect_six_face(collector, options.simulate ? &simulator : nullptr,
                              options);
  } else {
    result = monitor(collector);
  }

  simulator.stop();
  if (livox_started) {
    LivoxLidarSdkUninit();
  }
  return result;
}
