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
    reactToGameEvent(detectGameEvent(prev, last_game_status_),
                     match_start_time_ns);
  }

  EnvironmentContext::GameEvent EnvironmentContext::detectGameEvent(
      uint8_t prev, uint8_t current) {
    if (prev != 4 && current == 4) {
      return GameEvent::STARTED;
    }
    if (prev == 4 && current != 4) {
      return GameEvent::OVER;
    }
    return GameEvent::NONE;
  }

  void EnvironmentContext::reactToGameEvent(GameEvent event, int64_t now_ns) {
    switch (event) {
      case GameEvent::STARTED:
        match_started_ = true;
        match_start_time_ = now_ns;
        is_game_started_ = true;
        break;
      case GameEvent::OVER:
        match_started_ = false;
        match_start_time_ = 0;
        default_spin_latched_ = false;
        is_game_started_ = false;
        is_game_over_ = true;
        break;
      case GameEvent::NONE:
        break;
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

  static bool isRecent(Stamp last, int64_t now_ns, double hold_sec) {
    const int64_t last_ns = toFullNanos(last.sec, last.nanosec);
    return (last_ns != 0)
        && (static_cast<double>(now_ns - last_ns) * 1e-9 <= hold_sec);
  }

  void EnvironmentContext::updateTracking(Stamp now, const Snapshot& snapshot) {
    if (snapshot.enemy) {
      last_enemy_seen_ = now;
    }
    if (snapshot.rs.is_hp_deduced) {
      last_attacked_ = now;
    }
  }

  Snapshot EnvironmentContext::buildSnapshot(Stamp now) {
    std::lock_guard<std::mutex> lock(mtx_);

    Snapshot snapshot;
    snapshot.rs = last_robot_status_;
    snapshot.match_started = match_started_;
    snapshot.match_start_time = toStamp(match_start_time_);
    snapshot.state = state_;
    snapshot.armors = last_armors_;
    snapshot.target_opt = last_target_opt_;

    if (!snapshot.armors.armors.empty() || snapshot.target_opt.has_value()) {
      snapshot.enemy = detectEnemy(snapshot.armors, snapshot.target_opt);
    }

    const int64_t now_ns = toFullNanos(now.sec, now.nanosec);
    snapshot.enemy_recent = isRecent(last_enemy_seen_, now_ns,
                                     attack_hold_sec_);
    snapshot.attacked_recent = isRecent(last_attacked_, now_ns,
                                        attacked_hold_sec_);

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
