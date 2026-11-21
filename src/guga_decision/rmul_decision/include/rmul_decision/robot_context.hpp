#pragma once

#include <chrono>
#include "decision_type.hpp"

namespace rmul_decision {

  struct RobotContext {
    // 比赛状态
    bool game_started{false};
    double game_time{0.0};

    // 机器人状态
    int hp{0};

    // 位置信息
    Points current_point{Points::UNKNOWN};
    Points target_point{Points::UNKNOWN};

    // 点位驻留时间
    double stay_duration{0.0};

    // 导航状态
    NavigationStatus navigation_status{NavigationStatus::IDLE};

    // 导航结果
    bool navigation_success{false};

    // 返航状态
    bool go_home_success{false};
  };

}  // namespace rmul_decision