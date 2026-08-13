#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace serial_driver {

// ===================== BR 帧协议常量 =====================

/// BR 帧头："BR"。
inline constexpr uint8_t FRAME_HEADER1{0x42};
inline constexpr uint8_t FRAME_HEADER2{0x52};

/// BR 命令码：运动控制帧和裁判系统帧。
inline constexpr uint8_t COMMAND_CODE_MOTION{0xCD};
inline constexpr uint8_t COMMAND_CODE_REFEREE{0xD1};

/// SOF(2B) + CMD(1B) + LEN(1B) + CRC8(1B)。
inline constexpr size_t FRAME_MIN_SIZE{5};

/// CRC8 算法参数，属于线协议定义。
inline constexpr uint8_t CRC8_POLYNOMIAL{0x31};
inline constexpr uint8_t CRC8_INIT{0xFF};

// ===================== BR 协议 payload 尺寸 =====================

/// BR 协议运动控制帧 payload 大小
inline constexpr size_t MOTION_PAYLOAD_SIZE{17};

/// BR 协议裁判系统帧 payload 大小
inline constexpr size_t REFEREE_PAYLOAD_SIZE{13};

// ===================== BR 协议 payload 类型 =====================

/// BR 协议运动控制帧 payload
using MotionPayload = std::array<uint8_t, MOTION_PAYLOAD_SIZE>;

/// BR 协议裁判系统帧 payload
using RefereePayload = std::array<uint8_t, REFEREE_PAYLOAD_SIZE>;

// ===================== 运动帧 payload 字段偏移 =====================

/** @brief PC→MCU 下发字段偏移（17 字节运动帧）。 */
namespace downlink_offset {

inline constexpr size_t VX{0};           // float LE 云台系 x 线速度 × scale
inline constexpr size_t VY{4};           // float LE 云台系 y 线速度 × scale
inline constexpr size_t WZ_NEG{8};       // float LE 角速度 z 取反

}  // namespace downlink_offset

/** @brief MCU→PC 上传字段偏移（从同一 17 字节帧不同位置解析）。 */
namespace uplink_offset {

inline constexpr size_t YAW_DIFF{0};       // float LE   云台 yaw 角度差
inline constexpr size_t ENEMY_X{4};       // float LE 敌方 x 坐标
inline constexpr size_t ENEMY_Y{8};       // float LE 敌方 y 坐标

}  // namespace uplink_offset

// ===================== 裁判帧 payload 字段偏移 =====================

namespace referee_offset {

inline constexpr size_t GAME_PROGRESS{0};
inline constexpr size_t CURRENT_HP{1};
inline constexpr size_t AMMO_17MM{3};
inline constexpr size_t BARREL_HEAT{5};
inline constexpr size_t RFID_STATUS{9};

}  // namespace referee_offset

}  // namespace serial_driver
