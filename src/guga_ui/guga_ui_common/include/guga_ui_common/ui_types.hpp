#ifndef GUGA_UI_COMMON_UI_TYPES_HPP
#define GUGA_UI_COMMON_UI_TYPES_HPP
/**
 * @file ui_types.hpp
 * @brief guga_ui 共享内存中传输的所有数据类型定义。
 *
 * 所有结构体均为 POD（Plain Old Data），可安全地 memcpy 到共享内存。
 * 每个结构体按 64 字节对齐，避免跨 cache line 的 false sharing。
 *
 * 使用时：模块端在 publish 之后调用 ShmWriter::write() 写入对应结构体；
 *        UI 端通过 ShmReader 读取并渲染。
 */

#include <cstdint>

namespace guga_ui {

// ==================== 机器人状态 ====================

/**
 * @brief 机器人性能体系数据，来源于裁判系统协议 0x0201/0x0202/0x0208。
 *
 * 写入端: serial_driver_node (publishRefereeData)
 * 读取频率: UI 渲染帧率 (30–60Hz)
 */
struct alignas(64) UiRobotStatus {
  uint16_t current_hp{};
  uint16_t maximum_hp{};
  uint16_t shooter_17mm_1_barrel_heat{};
  uint16_t shooter_barrel_heat_limit{};
  uint16_t projectile_allowance_17mm{};
  uint16_t remaining_gold_coin{};
  uint16_t shooter_barrel_cooling_value{};
  uint8_t robot_id{};
  uint8_t robot_level{};
  uint8_t armor_id{};
  uint8_t hp_deduction_reason{};
  bool is_hp_deduced{false};

  // 填充至 64 字节对齐
  uint8_t _pad[43]{};
};

static_assert(sizeof(UiRobotStatus) == 64,
              "UiRobotStatus must be 64 bytes for cache-line alignment");
static_assert(std::is_trivially_copyable_v<UiRobotStatus>,
              "UiRobotStatus must be trivially copyable for shm memcpy");

// ==================== 比赛状态 ====================

/**
 * @brief 比赛进程与剩余时间，来源于裁判系统协议 0x0001。
 *
 * 写入端: serial_driver_node (publishRefereeData)
 */
struct alignas(64) UiGameStatus {
  /// 比赛阶段: 0=未开始 1=准备 2=自检 3=倒计时 4=比赛中 5=结算
  uint8_t game_progress{};
  /// 当前阶段剩余时间，秒
  int32_t stage_remain_time{};
  /// 已比赛时长，秒（上位机推算）
  double elapsed_sec{};

  uint8_t _pad[51]{};
};

static_assert(sizeof(UiGameStatus) == 64);
static_assert(std::is_trivially_copyable_v<UiGameStatus>);

// ==================== RFID 状态 ====================

/**
 * @brief RFID 增益点状态，来源于裁判系统协议 0x0209。
 *
 * 写入端: serial_driver_node (publishRefereeData)
 */
struct alignas(64) UiRfidStatus {
  bool base_gain_point{false};
  bool central_highland_gain_point{false};
  bool enemy_central_highland_gain_point{false};
  bool friendly_trapezoidal_highland_gain_point{false};
  bool enemy_trapezoidal_highland_gain_point{false};
  bool friendly_fly_ramp_front_gain_point{false};
  bool friendly_fly_ramp_back_gain_point{false};
  bool enemy_fly_ramp_front_gain_point{false};
  bool enemy_fly_ramp_back_gain_point{false};
  bool friendly_central_highland_lower_gain_point{false};
  bool friendly_central_highland_upper_gain_point{false};
  bool enemy_central_highland_lower_gain_point{false};
  bool enemy_central_highland_upper_gain_point{false};
  bool friendly_highway_lower_gain_point{false};
  bool friendly_highway_upper_gain_point{false};
  bool enemy_highway_lower_gain_point{false};
  bool enemy_highway_upper_gain_point{false};
  bool friendly_fortress_gain_point{false};
  bool friendly_outpost_gain_point{false};
  bool friendly_supply_zone_non_exchange{false};
  bool friendly_supply_zone_exchange{false};
  bool friendly_big_resource_island{false};
  bool enemy_big_resource_island{false};
  bool center_gain_point{false};

  uint8_t _pad[40]{};
};

static_assert(sizeof(UiRfidStatus) == 64);
static_assert(std::is_trivially_copyable_v<UiRfidStatus>);

// ==================== 决策状态 ====================

/**
 * @brief 决策系统的当前状态、底盘模式与导航目标。
 *
 * 写入端: simple_decision (executeAction)
 */
struct alignas(64) UiDecision {
  /// 决策状态: 0=DEFAULT 1=ATTACK 2=SUPPLY
  uint8_t state{};
  /// 底盘模式: 0=CHASSIS_FOLLOWED 1=LITTLE_TES 2=GO_HOME
  uint8_t chassis_mode{};
  bool should_publish_goal{false};
  bool default_spin_latched{false};

  /// 导航目标位姿
  double target_x{};
  double target_y{};
  double target_yaw{};

  /// 机器人当前位姿
  double robot_x{};
  double robot_y{};
  double robot_yaw{};

  /// 决策计算耗时 (微秒)
  int64_t compute_us{};

  uint8_t _pad[15]{};
};

static_assert(sizeof(UiDecision) == 64);
static_assert(std::is_trivially_copyable_v<UiDecision>);

// ==================== 敌方信息 ====================

/**
 * @brief 视觉检测的敌方装甲板和追踪目标。
 *
 * 写入端: simple_decision (onArmors / onTarget 回调)
 */
struct alignas(64) UiEnemy {
  /// 检测到的装甲板数量
  uint8_t armor_count{};
  /// 是否有追踪目标
  bool tracking{false};
  /// 是否有敌方（决策判断结果）
  bool enemy_detected{false};

  /// 追踪目标位置
  double target_x{};
  double target_y{};
  double target_z{};

  /// 最优攻击目标位置
  double attack_x{};
  double attack_y{};

  /// 敌方最后一次出现时间 (秒)
  double last_seen_sec{};

  uint8_t _pad[30]{};
};

static_assert(sizeof(UiEnemy) == 64);
static_assert(std::is_trivially_copyable_v<UiEnemy>);

// ==================== 里程计 ====================

/**
 * @brief 机器人里程计位姿与速度，来源于 Point-LIO。
 *
 * 写入端: point_lio (publish_odometry)
 */
struct alignas(64) UiOdom {
  double x{};
  double y{};
  double z{};
  double roll{};
  double pitch{};
  double yaw{};
  double vx{};
  double vy{};
  double vz{};

  uint8_t _pad[8]{};
};

static_assert(sizeof(UiOdom) == 64);
static_assert(std::is_trivially_copyable_v<UiOdom>);

// ==================== 云台偏航 ====================

/**
 * @brief 云台 yaw 轴角度差和下位机上报的角速度。
 *
 * 写入端: serial_driver_node (decodeYaw / decodeTESspeed)
 */
struct alignas(64) UiYaw {
  /// 下位机上报的 yaw 角度差（弧度）
  double yaw_diff{};
  /// 下位机上报的 TES 角速度
  double tes_angular_z{};

  uint8_t _pad[48]{};
};

static_assert(sizeof(UiYaw) == 64);
static_assert(std::is_trivially_copyable_v<UiYaw>);

// ==================== 导航路径 ====================

/// 单条导航路径最多存储的点数
static constexpr size_t UI_PATH_MAX_POINTS{256};

/**
 * @brief 导航规划路径，最多 256 个路径点。
 *
 * 写入端: 未来可接入 nav2 的 plan 回调
 */
struct alignas(64) UiPath {
  /// 实际路径点数量
  uint32_t count{};
  /// 时间戳
  double stamp_sec{};

  /// 路径点坐标数组
  double x[UI_PATH_MAX_POINTS]{};
  double y[UI_PATH_MAX_POINTS]{};

  uint8_t _pad[44]{};
};

static_assert(offsetof(UiPath, count) == 0);
static_assert(offsetof(UiPath, x) % 8 == 0);
static_assert(std::is_trivially_copyable_v<UiPath>);

// ==================== 槽位枚举 ====================

/**
 * @brief 共享内存中的 slot ID 分配。
 *
 * 每个算法模块写入时用固定 slot_id，UI 读取时按 slot_id 索引。
 * 新增 slot 必须追加在末尾（不删除/重排已有值），保证向后兼容。
 */
enum class UiSlotId : uint32_t {
  ROBOT_STATUS = 0,
  GAME_STATUS = 1,
  RFID_STATUS = 2,
  DECISION = 3,
  ENEMY = 4,
  ODOM = 5,
  YAW = 6,
  PATH = 7,

  /// 槽位总数（新增后必须更新此值）
  COUNT = 8
};

}  // namespace guga_ui

#endif  // GUGA_UI_COMMON_UI_TYPES_HPP
