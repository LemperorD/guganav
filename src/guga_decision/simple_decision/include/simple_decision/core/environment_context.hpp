#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <algorithm>
#include <cstdint>

#include "guga_interfaces/msg/armors.hpp"
#include "guga_interfaces/msg/target.hpp"
#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "types.hpp"

namespace simple_decision {

  struct Readiness {
    enum class Status : std::uint8_t {
      READY,
      NO_RS,
      NO_GS,
      NOT_STARTED,
      IN_DELAY
    };
    Status status = Status::NOT_STARTED;
    double elapsed = 0.0;  // 仅供 Log 使用
  };

  /// Environment state aggregation layer
  /// Translates low-level ROS messages into high-level business facts
  class EnvironmentContext {
  public:
    explicit EnvironmentContext() = default;

    explicit EnvironmentContext(const ContextConfig& context_config);

    void onRobotStatus(const RobotStatus& robot_status);
    void onArmors(const Armors& armors);
    void onTarget(const Target& target);
    void onGameStatus(GameStatus game_status, int64_t match_start_time_ns);

    bool isGameStarted() const;
    bool isGameOver() const;
    void resetGameOver();

    bool getRobotPoseMap(double& x, double& y, double& yaw);

    Snapshot getSnapshot(Stamp now);

    bool isStatusBad(const RobotStatus& robotstatus) const;
    bool isStatusRecovered(const RobotStatus& robotstatus) const;

    bool detectEnemy(const Armors& armors,
                     const std::optional<Target>& target_opt) const;

    bool changeState(State state);

    void setChassisMode(ChassisMode mode);
    Readiness checkReadiness(int64_t now) const;
    void updatePose(double x, double y, double z);
    bool isNearRobotPose(double target_x, double target_y,
                         double tolerance) const;

  private:
    const ContextConfig config_;
    mutable std::mutex mtx_;
    bool has_robot_status_{false};
    bool match_started_{false};
    bool has_pose_{false};
    bool has_game_status_{false};
    bool default_spin_latched_{false};
    bool is_game_started_{false};
    bool is_game_over_{false};
    bool has_armors_{false};

    double attack_hold_sec_{1.5};
    double attacked_hold_sec_{1.5};

    Stamp last_enemy_seen_{0, 0};
    Stamp last_attacked_{0, 0};

    RobotStatus last_robot_status_{};
    int64_t match_start_time_{};
    Armors last_armors_{};
    std::optional<Target> last_target_opt_;
    State state_{State::DEFAULT};
    uint8_t last_game_status_{0};

    double x_{0.0}, y_{0.0}, yaw_{0.0};
  };

}  // namespace simple_decision
