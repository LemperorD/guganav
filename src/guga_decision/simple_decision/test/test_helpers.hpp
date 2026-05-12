#pragma once

#include "simple_decision/core/types.hpp"

namespace simple_decision {

  // ── constants ──
  constexpr double kDefaultX = 2.0;
  constexpr double kDefaultY = 0.5;
  constexpr double kDefaultYaw = 0.0;
  constexpr double kSupplyX = 0.0;
  constexpr double kSupplyY = 0.0;
  constexpr double kSupplyYaw = 0.0;
  constexpr int kHpEnterSupply = 120;
  constexpr int kHpExitSupply = 300;
  constexpr int kAmmoMin = 0;
  constexpr double kCombatMaxDist = 8.0;
  constexpr double kArriveTol = 0.30;
  constexpr double kSpinKeepTol = 0.80;
  constexpr double kStartDelaySec = 5.0;

  inline ContextConfig DefaultConfig() {
    return {kHpEnterSupply, kHpExitSupply, kAmmoMin,   kCombatMaxDist, false,
            kStartDelaySec, kDefaultX,     kDefaultY,  kDefaultYaw,    kSupplyX,
            kSupplyY,       kSupplyYaw,    kArriveTol, kSpinKeepTol};
  }

  inline ContextConfig RequireGameRunningConfig() {
    auto c = DefaultConfig();
    c.require_game_running = true;
    return c;
  }

  inline RobotStatus HealthyRobotStatus() {
    RobotStatus rs;
    rs.current_hp = 500;
    rs.projectile_allowance_17mm = 120;
    rs.is_hp_deduced = false;
    return rs;
  }

  inline RobotStatus LowHpRobotStatus() {
    RobotStatus rs;
    rs.current_hp = 100;
    rs.projectile_allowance_17mm = 120;
    rs.is_hp_deduced = false;
    return rs;
  }

  inline RobotStatus LowAmmoRobotStatus() {
    RobotStatus rs;
    rs.current_hp = 500;
    rs.projectile_allowance_17mm = 0;
    rs.is_hp_deduced = false;
    return rs;
  }

  inline RobotStatus AttackedRobotStatus() {
    RobotStatus rs = HealthyRobotStatus();
    rs.is_hp_deduced = true;
    return rs;
  }

  inline GameStatus RunningGameStatus() {
    GameStatus gs;
    gs.game_progress = GameStatus::RUNNING;
    return gs;
  }

  inline Armors ArmorInRange() {
    Armors armors;
    Armor a;
    a.pose.position.x = 2.0;
    a.pose.position.y = 0.0;
    a.pose.position.z = 0.0;
    armors.armors.push_back(a);
    return armors;
  }

  inline Armors ArmorOutOfRange() {
    Armors armors;
    Armor a;
    a.pose.position.x = 100.0;
    a.pose.position.y = 0.0;
    a.pose.position.z = 0.0;
    armors.armors.push_back(a);
    return armors;
  }

  inline Target TrackingTarget() {
    Target t;
    t.tracking = true;
    t.position.x = 1.0;
    t.position.y = 0.0;
    t.position.z = 0.0;
    t.yaw = 1.57;
    return t;
  }

  inline Target NonTrackingTarget() {
    Target t;
    t.tracking = false;
    return t;
  }

  inline Stamp MakeStamp(int32_t sec, uint32_t nanosec) {
    return {sec, nanosec};
  }

  inline Snapshot HealthyNotAttackedSnapshot() {
    Snapshot s{};
    s.robotstatus = HealthyRobotStatus();
    s.state = State::DEFAULT;
    return s;
  }

  inline Snapshot EnemyRecentSnapshot() {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.armors = ArmorInRange();
    s.enemy = true;
    s.enemy_recent = true;
    return s;
  }

  inline Snapshot AttackedRecentSnapshot() {
    Snapshot s = HealthyNotAttackedSnapshot();
    s.attacked_recent = true;
    return s;
  }

}  // namespace simple_decision
