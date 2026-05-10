#pragma once

#include "simple_decision/core/environment_context.hpp"
#include "simple_decision/core/types.hpp"

namespace simple_decision {
  class Decision {
  public:
    explicit Decision(const ContextConfig& context_config);
    [[nodiscard]] DecisionAction computeAction(const Snapshot& snapshot) const;
    [[nodiscard]] bool isStatusRecovered(const RobotStatus& rs) const;
    [[nodiscard]] bool isStatusBad(const RobotStatus& rs) const;
    [[nodiscard]] static bool buildAttackGoal(
        Snapshot& snapshot, const Armors& armors,
        const std::optional<Target>& target_opt);

  private:
    [[nodiscard]] DecisionAction supplyAction() const;
    [[nodiscard]] DecisionAction attackAction(const Snapshot& s) const;
    [[nodiscard]] DecisionAction defaultAction(const Snapshot& s) const;

    ContextConfig config_;
  };
}  // namespace simple_decision
