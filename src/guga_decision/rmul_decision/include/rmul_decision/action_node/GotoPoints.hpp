#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "guga_decision_common/decision_input_hub.hpp"
#include "guga_decision_common/goal_controller.hpp"
#include "guga_decision_common/waypoint_navigator.hpp"

class PointSelector : public BT::StatefulActionNode {
public:
  PointSelector(
      const std::string& name, const BT::NodeConfig& config,
      std::shared_ptr<guga_decision_common::DecisionInputHub> input_hub,
      std::shared_ptr<guga_decision_common::GoalController> goal_controller);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::vector<guga_decision_common::Waypoint2D> buildRoute(
      int destination, const std::string& strategy) const;

  std::optional<std::vector<guga_decision_common::Waypoint2D>> routeFromPorts();

  static std::vector<guga_decision_common::Waypoint2D> toRoute(
      const std::vector<std::pair<double, double>>& points);

  std::shared_ptr<guga_decision_common::DecisionInputHub> input_hub_;
  std::shared_ptr<guga_decision_common::GoalController> goal_controller_;
  guga_decision_common::WaypointNavigator navigator_;

  // 坐标由点位配置填充；一个数组可以包含多个航点。
  std::vector<std::pair<double, double>> point_1_{
      {0.0, 0.0}
  };
  std::vector<std::pair<double, double>> point_2_{
      {2.0, 1.0}
  };
  std::vector<std::pair<double, double>> point_3_{
      {4.0, 2.0}
  };
  std::vector<std::pair<double, double>> point_4_{
      {2.0, 5.0}
  };
  std::vector<std::pair<double, double>> point_5_{
      {-1.0, 6.0}
  };
  std::vector<std::pair<double, double>> tunnel_1_{
      {-1.0, 2.5},
      {-1.0, 0.5},
      { 0.0, 0.0}
  };
  std::vector<std::pair<double, double>> tunnel_4_{
      {-1.0, 0.5},
      {-1.0, 2.5},
      { 2.0, 5.0}
  };
  std::vector<std::pair<double, double>> tunnel_5_{
      {-1.0, 0.5},
      {-1.0, 2.5},
      {-1.0, 6.0}
  };
};
