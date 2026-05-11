#include "simple_decision/core/decision.hpp"

#include <cmath>

namespace simple_decision {

  Decision::Decision(const ContextConfig& context_config)
      : config_(context_config) {
  }

  // ── public ──

  DecisionAction Decision::computeAction(const Snapshot& snapshot) const {
    if (snapshot.state == State::SUPPLY && !isStatusRecovered(snapshot.rs)) {
      return supplyAction();
    }
    if (isStatusBad(snapshot.rs)) {
      return supplyAction();
    }
    if (snapshot.enemy_recent) {
      return attackAction(snapshot);
    }
    return defaultAction(snapshot);
  }

  bool Decision::isStatusRecovered(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp >= config_.hp_exit_supply) && (ammo > config_.ammo_min);
  }

  bool Decision::isStatusBad(const RobotStatus& rs) const {
    const int hp = static_cast<int>(rs.current_hp);
    const int ammo = static_cast<int>(rs.projectile_allowance_17mm);
    return (hp < config_.hp_enter_supply) || (ammo <= config_.ammo_min);
  }

  std::optional<Pose2D> Decision::findAttackPosition(
      const Armors& armors, const std::optional<Target>& target_opt) {
    if (target_opt.has_value() && target_opt->tracking) {
      return Pose2D{target_opt->position.x, target_opt->position.y,
                    target_opt->yaw};
    }

    if (armors.armors.empty()) {
      return std::nullopt;
    }

    const auto* best = &armors.armors.front();
    double best_dist = 1e18;
    for (const auto& a : armors.armors) {
      const double x = a.pose.position.x;
      const double y = a.pose.position.y;
      const double z = a.pose.position.z;
      const double dist = std::sqrt((x * x) + (y * y) + (z * z));
      if (dist < best_dist) {
        best_dist = dist;
        best = &a;
      }
    }

    return Pose2D{best->pose.position.x, best->pose.position.y, 0.0};
  }

  // ── private ──

  DecisionAction Decision::supplyAction() const {
    DecisionAction action;
    action.should_publish_goal = true;
    action.next_state = State::SUPPLY;
    action.chassis_mode = ChassisMode::CHASSIS_FOLLOWED;
    action.target_x = config_.supply_x;
    action.target_y = config_.supply_y;
    action.target_yaw = config_.supply_yaw;
    return action;
  }

  DecisionAction Decision::attackAction(const Snapshot& s) const {
    DecisionAction action;
    action.should_publish_goal = true;
    action.next_state = State::ATTACK;
    action.chassis_mode = s.attacked_recent ? ChassisMode::LITTLE_TES
                                            : ChassisMode::CHASSIS_FOLLOWED;
    action.target_x = config_.default_x;
    action.target_y = config_.default_y;
    action.target_yaw = config_.default_yaw;

    if (auto target = findAttackPosition(s.armors, s.target_opt)) {
      action.target_x = target->x;
      action.target_y = target->y;
      action.target_yaw = target->yaw;
    }
    return action;
  }

  DecisionAction Decision::defaultAction(const Snapshot& s) const {
    DecisionAction action;
    action.should_publish_goal = true;
    action.next_state = State::DEFAULT;
    action.target_x = config_.default_x;
    action.target_y = config_.default_y;
    action.target_yaw = config_.default_yaw;
    action.default_spin_latched = s.at_center;

    if (!s.in_center_keep_spin) {
      action.default_spin_latched = false;
    }

    action.chassis_mode = s.attacked_recent || action.default_spin_latched
                            ? ChassisMode::LITTLE_TES
                            : ChassisMode::CHASSIS_FOLLOWED;
    return action;
  }

}  // namespace simple_decision
