#include "guga_decision_common/waypoint_navigator.hpp"

#include <cmath>
#include <utility>

namespace guga_decision_common
{

WaypointNavigator::WaypointNavigator(
  SendGoal send_goal, CancelGoal cancel_goal)
: send_goal_(std::move(send_goal)), cancel_goal_(std::move(cancel_goal))
{
}

WaypointNavigatorStatus WaypointNavigator::start(
  std::vector<Waypoint2D> route)
{
  if (running() && cancel_goal_) {
    cancel_goal_();
  }

  route_.assign(route.begin(), route.end());
  total_count_ = route_.size();
  completed_count_ = 0;

  if (route_.empty()) {
    succeed();
    return status_;
  }

  if (!validWaypoint(route_.front())) {
    fail();
    return status_;
  }

  status_ = WaypointNavigatorStatus::RUNNING;
  if (!sendCurrentGoal()) {
    fail();
  }
  return status_;
}

WaypointNavigatorStatus WaypointNavigator::update(bool arrived)
{
  if (status_ != WaypointNavigatorStatus::RUNNING) {
    return status_;
  }

  if (!arrived || route_.empty()) {
    return status_;
  }

  route_.pop_front();
  ++completed_count_;

  if (route_.empty()) {
    succeed();
    return status_;
  }

  if (!validWaypoint(route_.front())) {
    fail();
    return status_;
  }

  if (!sendCurrentGoal()) {
    fail();
  }
  return status_;
}

void WaypointNavigator::halt()
{
  if (running() && cancel_goal_) {
    cancel_goal_();
  }
  route_.clear();
  status_ = WaypointNavigatorStatus::IDLE;
  total_count_ = 0;
  completed_count_ = 0;
}

std::optional<Waypoint2D> WaypointNavigator::currentWaypoint() const
{
  if (route_.empty()) {
    return std::nullopt;
  }
  return route_.front();
}

bool WaypointNavigator::validWaypoint(const Waypoint2D& waypoint) const
{
  return std::isfinite(waypoint.x) &&
         std::isfinite(waypoint.y) &&
         std::isfinite(waypoint.yaw);
}

bool WaypointNavigator::sendCurrentGoal()
{
  return send_goal_ && !route_.empty() && send_goal_(route_.front());
}

void WaypointNavigator::fail()
{
  route_.clear();
  status_ = WaypointNavigatorStatus::FAILED;
}

void WaypointNavigator::succeed()
{
  route_.clear();
  status_ = WaypointNavigatorStatus::SUCCEEDED;
}

}  // namespace guga_decision_common
