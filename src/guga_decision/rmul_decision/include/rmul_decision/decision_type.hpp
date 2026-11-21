#pragma once

#include <cstdint>

namespace rmul_decision {
  enum class StrategyMode : uint8_t {
    UNKNOWN = 0,
    SUPPRESS,
    OCCUPATION,
    PUSHFORWARD
  };
  enum class Points : uint8_t {
    UNKNOWN = 0,
    POINT_1,
    POINT_2,
    POINT_3,
    POINT_4,
    POINT_5
  };
  enum class NavigationStatus : uint8_t {
    IDLE = 0,
    RUNNING,
    SUCCESS,
    FAILURE,
    CANCELED
  };

  // 决策类型
  enum class DecisionType : uint8_t { NONE = 0, NAVIGATE, STAY, GO_HOME, WAIT };

};  // namespace rmul_decision
