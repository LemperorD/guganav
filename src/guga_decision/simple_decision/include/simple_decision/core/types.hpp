#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace simple_decision {
  using nanoseconds = int64_t;

  /// Chassis movement mode
  enum class ChassisMode : uint8_t {
    CHASSIS_FOLLOWED,
    LITTLE_TES,
    GO_HOME,
  };

  enum class State : uint8_t {
    DEFAULT,
    ATTACK,
    SUPPLY,
  };

  struct ContextConfig {
    int hp_enter_supply{120};
    int hp_exit_supply{300};
    int ammo_min{0};
    double combat_max_distance{8.0};
    bool require_game_running{false};
    double start_delay_sec{5.0};
    double default_x{0};
    double default_y{0};
    double default_yaw{0};
    double supply_x{0};
    double supply_y{0};
    double supply_yaw{0};
    double default_arrive_xy_tol{0.30};
    double default_spin_keep_xy_tol{0.80};
  };

  struct DecisionAction {
    State next_state = State::DEFAULT;
    ChassisMode chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_yaw = 0.0;
    bool should_publish_goal = false;
    bool default_spin_latched = false;
  };

  struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
  };

  struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct Pose2D {
    double x;
    double y;
    double yaw;
  };
  struct Pose3D {
    Position position;
    Quaternion orientation;
  };

  struct RobotStatus {
    uint8_t robot_id = 0;
    uint8_t robot_level = 0;
    uint16_t current_hp = 0;
    uint16_t maximum_hp = 0;

    uint16_t shooter_barrel_cooling_value = 0;
    uint16_t shooter_barrel_heat_limit = 0;
    uint16_t shooter_17mm_1_barrel_heat = 0;

    Pose3D robot_pos;

    uint8_t armor_id = 0;
    uint8_t hp_deduction_reason = 0;
    uint16_t projectile_allowance_17mm = 0;
    uint16_t remaining_gold_coin = 0;
    bool is_hp_deduced = false;

    static constexpr uint8_t ARMOR_HIT = 0U;
    static constexpr uint8_t SYSTEM_OFFLINE = 1U;
    static constexpr uint8_t OVER_SHOOT_SPEED = 2U;
    static constexpr uint8_t OVER_HEAT = 3U;
    static constexpr uint8_t OVER_POWER = 4U;
    static constexpr uint8_t ARMOR_COLLISION = 5U;
  };

  struct GameStatus {
    uint8_t game_progress = 0;
    int32_t stage_remain_time = 0;

    static constexpr uint8_t NOT_START = 0U;
    static constexpr uint8_t PREPARATION = 1U;
    static constexpr uint8_t SELF_CHECKING = 2U;
    static constexpr uint8_t COUNT_DOWN = 3U;
    static constexpr uint8_t RUNNING = 4U;
    static constexpr uint8_t GAME_OVER = 5U;
  };

  struct Stamp {
    int32_t sec;
    uint32_t nanosec;
  };

  inline int64_t toFullNanos(int32_t sec, uint32_t nanosec) {
    return (static_cast<int64_t>(sec) * 1'000'000'000LL) + nanosec;
  }

  inline Stamp toStamp(int64_t full_nanos) {
    return {static_cast<int32_t>(full_nanos / 1'000'000'000LL),
            static_cast<uint32_t>(full_nanos % 1'000'000'000LL)};
  }

  struct Header {
    Stamp stamp;
    std::string frame_id;
  };

  struct Armor {
    Pose3D pose;
  };

  struct Armors {
    Header header;

    std::vector<Armor> armors;
  };

  struct Target {
    Header header;

    Position position;
    double yaw = 0.0;
    bool tracking = false;
  };

  struct Snapshot {
    bool has_attack_goal{false};
    bool match_started = false;
    bool enemy_recent = false;
    bool attacked_recent = false;
    bool at_center{false};
    bool in_center_keep_spin{false};
    bool default_spin_latched{false};
    bool enemy{false};
    double last_attack_yaw{0.0};
    RobotStatus rs;
    Stamp match_start_time{0, 0};
    Armors armors;
    std::optional<Target> target_opt;
    State state{State::DEFAULT};
    Position last_attack_position{0.0, 0.0, 0.0};
  };

}  // namespace simple_decision
