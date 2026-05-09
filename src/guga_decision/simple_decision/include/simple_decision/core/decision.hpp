#pragma once

#include "simple_decision/core/environment_context.hpp"
#include "simple_decision/core/types.hpp"

namespace simple_decision {
  class Decision {
  public:
    explicit Decision(const ContextConfig& context_config);
    DecisionAction computeAction(const Snapshot& snapshot) const;
    bool isStatusRecovered(const RobotStatus& rs) const;
    bool isStatusBad(const RobotStatus& rs) const;
    bool buildAttackGoal(Snapshot& snapshot, const Armors& armors,
                         const std::optional<Target>& target_opt) const;

  private:
    DecisionAction supplyAction() const;
    DecisionAction attackAction(const Snapshot& s) const;
    DecisionAction defaultAction(const Snapshot& s) const;

    const ContextConfig config;
  };
}  // namespace simple_decision
