#include "simple_decision/core/environment_context.hpp"

namespace simple_decision {

  EnvironmentContext::EnvironmentContext(const ContextConfig& context_config)
      : config_(context_config) {
  }

  void EnvironmentContext::onRobotStatus(const RobotStatus& robot_status) {
    std::lock_guard<std::mutex> lock(mtx_);

    last_robot_status_ = robot_status;
    has_robot_status_ = true;
  }

  void EnvironmentContext::onGameStatus(GameStatus game_status,
                                        int64_t match_start_time_ns) {
    std::lock_guard<std::mutex> lock(mtx_);

    const uint8_t prev = last_game_status_;
    last_game_status_ = game_status.game_progress;
    has_game_status_ = true;

    if (prev != 4 && last_game_status_ == 4) {
      match_started_ = true;
      match_start_time_ = match_start_time_ns;
      is_game_started_ = true;
    }

    // 从“比赛中(4)” -> “非比赛中”，复位，为下一局做准~备
    if (prev == 4 && last_game_status_ != 4) {
      match_started_ = false;
      match_start_time_ = 0;
      // 清掉上一局遗留的默认点小陀螺锁存
      default_spin_latched_ = false;
      is_game_started_ = false;
      is_game_over_ = true;
    }
  }

  bool EnvironmentContext::isGameStarted() const {
    return is_game_started_;
  }

  bool EnvironmentContext::isGameOver() const {
    return is_game_over_;
  }

  void EnvironmentContext::resetGameOver() {
    is_game_over_ = false;
  }

  void EnvironmentContext::onArmors(const Armors& armors) {
    std::lock_guard<std::mutex> lock(mtx_);

    last_armors_ = armors;
    has_armors_ = true;
  }

  void EnvironmentContext::onTarget(const Target& target) {
    std::lock_guard<std::mutex> lock(mtx_);

    last_target_opt_ = target;
  }

  bool EnvironmentContext::detectEnemy(
      const Armors& armors, const std::optional<Target>& target_opt) const {
    if (target_opt.has_value() && target_opt->tracking) {
      const double x = target_opt->position.x;
      const double y = target_opt->position.y;
      const double z = target_opt->position.z;
      const double dist = std::sqrt((x * x) + (y * y) + (z * z));
      return dist <= config_.combat_max_distance;
    }

    return std::any_of(armors.armors.begin(), armors.armors.end(),
                       [&](const auto& armor) {
                         const double dist = std::sqrt(
                             (armor.pose.position.x * armor.pose.position.x)
                             + (armor.pose.position.y * armor.pose.position.y)
                             + (armor.pose.position.z * armor.pose.position.z));
                         return dist <= config_.combat_max_distance;
                       });
  }

  Snapshot EnvironmentContext::getSnapshot(Stamp now) {
    std::lock_guard<std::mutex> lock(mtx_);

    Snapshot snapshot;
    snapshot.has_rs = has_robot_status_;
    if (snapshot.has_rs) {
      snapshot.rs = last_robot_status_;
    }
    snapshot.has_gs = has_game_status_;
    snapshot.match_started = match_started_;
    snapshot.match_start_time = toStamp(match_start_time_);
    snapshot.has_armors = has_armors_;
    snapshot.state = state_;
    if (snapshot.has_armors) {
      snapshot.armors = last_armors_;
    }
    snapshot.target_opt = last_target_opt_;

    if (snapshot.has_armors || snapshot.target_opt.has_value()) {
      snapshot.enemy = detectEnemy(snapshot.armors, snapshot.target_opt);
    }
    if (snapshot.enemy) {
      last_enemy_seen_ = now;
    }

    const int64_t now_ns = toFullNanos(now.sec, now.nanosec);
    const int64_t last_enemy_ns = toFullNanos(last_enemy_seen_.sec,
                                              last_enemy_seen_.nanosec);
    snapshot.enemy_recent = (last_enemy_ns != 0)
                         && (static_cast<double>(now_ns - last_enemy_ns) * 1e-9
                             <= attack_hold_sec_);

    if (snapshot.rs.is_hp_deduced) {
      last_attacked_ = now;
    }
    const int64_t last_attacked_ns = toFullNanos(last_attacked_.sec,
                                                last_attacked_.nanosec);
    snapshot.attacked_recent = (last_attacked_ns != 0)
                            && (static_cast<double>(now_ns - last_attacked_ns)
                                    * 1e-9
                                <= attacked_hold_sec_);

    snapshot.default_spin_latched = default_spin_latched_;
    snapshot.at_center = isNearRobotPose(config_.default_x, config_.default_y,
                                         config_.default_arrive_xy_tol);
    snapshot.in_center_keep_spin = isNearRobotPose(
        config_.default_x, config_.default_y, config_.default_spin_keep_xy_tol);

    return snapshot;
  }

  bool EnvironmentContext::isStatusBad(const RobotStatus& robotstatus) const {
    const int hp = static_cast<int>(robotstatus.current_hp);
    const int ammo = static_cast<int>(robotstatus.projectile_allowance_17mm);
    return (hp < config_.hp_enter_supply) || (ammo <= config_.ammo_min);
  }

  bool EnvironmentContext::changeState(State state) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (state_ == state) {
      return false;
    }
    state_ = state;
    return true;
  }

  Readiness EnvironmentContext::checkReadiness(nanoseconds now) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_robot_status_) {
      return {Readiness::Status::NO_RS};
    }
    if (config_.require_game_running) {
      if (!has_game_status_) {
        return {Readiness::Status::NO_GS};
      }
      if (!match_started_) {
        return {Readiness::Status::NOT_STARTED};
      }

      const double current_elapsed =
          static_cast<double>(now - match_start_time_) * 1e-9;
      if (current_elapsed < config_.start_delay_sec) {
        return {Readiness::Status::IN_DELAY, current_elapsed};
      }
    }

    return {Readiness::Status::READY};
  }

  void EnvironmentContext::updatePose(const double x, const double y,
                                      const double yaw) {
    std::lock_guard<std::mutex> lock(mtx_);
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    has_pose_ = true;
  }

  bool EnvironmentContext::isNearRobotPose(double target_x, double target_y,
                                           double tolerance) const {
    if (!has_pose_) {
      return false;
    }
    double dx = x_ - target_x;
    double dy = y_ - target_y;
    return std::hypot(dx, dy) <= tolerance;
  }

}  // namespace simple_decision
