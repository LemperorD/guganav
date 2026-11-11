#include "rmul_decision/action_node/GotoPoints.hpp"

#include <cmath>
#include <utility>

namespace
{

BT::NodeStatus toBtStatus(
  guga_decision_common::WaypointNavigatorStatus status)
{
  switch (status) {
    case guga_decision_common::WaypointNavigatorStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    case guga_decision_common::WaypointNavigatorStatus::SUCCEEDED:
      return BT::NodeStatus::SUCCESS;
    case guga_decision_common::WaypointNavigatorStatus::FAILED:
    case guga_decision_common::WaypointNavigatorStatus::IDLE:
    default:
      return BT::NodeStatus::FAILURE;
  }
}

}  // namespace

PointSelector::PointSelector(
  const std::string& name,
  const BT::NodeConfig& config,
  std::shared_ptr<guga_decision_common::DecisionInputHub> input_hub,
  std::shared_ptr<guga_decision_common::GoalController> goal_controller)
: BT::StatefulActionNode(name, config),
  input_hub_(std::move(input_hub)),
  goal_controller_(std::move(goal_controller)),
  navigator_(
    [this](const guga_decision_common::Waypoint2D& waypoint) {
      return goal_controller_ &&
             goal_controller_->sendGoal(waypoint, true);
    })
{
}

BT::PortsList PointSelector::providedPorts()
{
  return {
    BT::InputPort<int>("destination"),
    BT::InputPort<std::string>("strategy_mode"),
    BT::InputPort<double>("tolerance", 0.15, "到达航点的距离容差"),
    BT::InputPort<std::vector<double>>("route_x", "绝对航点 x 数组"),
    BT::InputPort<std::vector<double>>("route_y", "绝对航点 y 数组"),
    BT::InputPort<std::vector<double>>("route_yaw", "绝对航点 yaw 数组"),
  };
}

BT::NodeStatus PointSelector::onStart()
{
  if (!input_hub_ || !goal_controller_) {
    return BT::NodeStatus::FAILURE;
  }

  int destination = 0;
  std::string strategy;
  if (!getInput("destination", destination) ||
      !getInput("strategy_mode", strategy))
  {
    return BT::NodeStatus::FAILURE;
  }

  auto route = routeFromPorts();
  if (!route) {
    route = buildRoute(destination, strategy);
  }
  return toBtStatus(navigator_.start(std::move(*route)));
}

BT::NodeStatus PointSelector::onRunning()
{
  if (!input_hub_) {
    return BT::NodeStatus::FAILURE;
  }

  const auto current = navigator_.currentWaypoint();
  if (!current) {
    return toBtStatus(navigator_.status());
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  if (!input_hub_->lookupRobotPose(robot_x, robot_y, robot_yaw)) {
    return BT::NodeStatus::RUNNING;
  }

  double tolerance = 0.15;
  getInput("tolerance", tolerance);
  const bool arrived =
    std::hypot(robot_x - current->x, robot_y - current->y) <= tolerance;

  return toBtStatus(navigator_.update(arrived));
}

void PointSelector::onHalted()
{
  navigator_.halt();
}

std::vector<guga_decision_common::Waypoint2D> PointSelector::buildRoute(
  int destination, const std::string& strategy) const
{
  const std::vector<std::pair<double, double>>* points = nullptr;
  const std::vector<std::pair<double, double>>* tunnel = nullptr;

  switch (destination) {
    case 1:
      points = &point_1_;
      tunnel = &tunnel_1_;
      break;
    case 2:
      points = &point_2_;
      break;
    case 3:
      points = &point_3_;
      break;
    case 4:
      points = &point_4_;
      tunnel = &tunnel_4_;
      break;
    case 5:
      points = &point_5_;
      tunnel = &tunnel_5_;
      break;
    default:
      return {};
  }

  // 压制状态允许使用对应点位的隧道路线，其它状态使用普通点位路线。
  if (strategy == "Suppress" && tunnel && !tunnel->empty()) {
    return toRoute(*tunnel);
  }
  return points ? toRoute(*points) : std::vector<guga_decision_common::Waypoint2D>{};
}

std::optional<std::vector<guga_decision_common::Waypoint2D>>
PointSelector::routeFromPorts()
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> yaws;

  const bool has_x = static_cast<bool>(getInput("route_x", xs));
  const bool has_y = static_cast<bool>(getInput("route_y", ys));
  if (!has_x && !has_y) {
    return std::nullopt;
  }
  if (!has_x || !has_y || xs.empty() || xs.size() != ys.size()) {
    return std::vector<guga_decision_common::Waypoint2D>{};
  }

  if (static_cast<bool>(getInput("route_yaw", yaws)) &&
      !yaws.empty() &&
      yaws.size() != xs.size())
  {
    return std::vector<guga_decision_common::Waypoint2D>{};
  }

  std::vector<guga_decision_common::Waypoint2D> route;
  route.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    route.push_back({
      xs[i],
      ys[i],
      yaws.empty() ? 0.0 : yaws[i],
    });
  }
  return route;
}

std::vector<guga_decision_common::Waypoint2D> PointSelector::toRoute(
  const std::vector<std::pair<double, double>>& points)
{
  std::vector<guga_decision_common::Waypoint2D> route;
  route.reserve(points.size());
  for (const auto& [x, y] : points) {
    route.push_back({x, y, 0.0});
  }
  return route;
}
