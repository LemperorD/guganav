#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "guga_decision_common/waypoint_navigator.hpp"

using guga_decision_common::Waypoint2D;
using guga_decision_common::WaypointNavigator;
using guga_decision_common::WaypointNavigatorStatus;

TEST(WaypointNavigator, EmptyRouteSucceeds)
{
  int sent = 0;
  WaypointNavigator navigator(
    [&sent](const Waypoint2D&) {
      ++sent;
      return true;
    });

  EXPECT_EQ(
    navigator.start({}), WaypointNavigatorStatus::SUCCEEDED);
  EXPECT_EQ(sent, 0);
  EXPECT_EQ(navigator.remaining(), 0u);
}

TEST(WaypointNavigator, SingleWaypointCompletesAfterArrival)
{
  std::vector<Waypoint2D> sent;
  WaypointNavigator navigator(
    [&sent](const Waypoint2D& waypoint) {
      sent.push_back(waypoint);
      return true;
    });

  EXPECT_EQ(
    navigator.start({{1.0, 2.0, 0.3}}),
    WaypointNavigatorStatus::RUNNING);
  EXPECT_EQ(sent.size(), 1u);
  EXPECT_DOUBLE_EQ(sent[0].x, 1.0);
  EXPECT_DOUBLE_EQ(sent[0].y, 2.0);
  EXPECT_EQ(navigator.remaining(), 1u);

  EXPECT_EQ(
    navigator.update(false), WaypointNavigatorStatus::RUNNING);
  EXPECT_EQ(
    navigator.update(true), WaypointNavigatorStatus::SUCCEEDED);
  EXPECT_EQ(navigator.remaining(), 0u);
  EXPECT_EQ(navigator.completedCount(), 1u);
}

TEST(WaypointNavigator, SendsWaypointsInOrder)
{
  std::vector<Waypoint2D> sent;
  WaypointNavigator navigator(
    [&sent](const Waypoint2D& waypoint) {
      sent.push_back(waypoint);
      return true;
    });

  EXPECT_EQ(
    navigator.start({{1.0, 1.0, 0.0},
                     {2.0, 2.0, 0.0},
                     {3.0, 3.0, 0.0}}),
    WaypointNavigatorStatus::RUNNING);

  EXPECT_EQ(
    navigator.update(true), WaypointNavigatorStatus::RUNNING);
  EXPECT_EQ(
    navigator.update(true), WaypointNavigatorStatus::RUNNING);
  EXPECT_EQ(
    navigator.update(true), WaypointNavigatorStatus::SUCCEEDED);

  ASSERT_EQ(sent.size(), 3u);
  EXPECT_DOUBLE_EQ(sent[0].x, 1.0);
  EXPECT_DOUBLE_EQ(sent[1].x, 2.0);
  EXPECT_DOUBLE_EQ(sent[2].x, 3.0);
  EXPECT_EQ(navigator.completedCount(), 3u);
}

TEST(WaypointNavigator, SendFailureStopsRoute)
{
  WaypointNavigator navigator(
    [](const Waypoint2D&) {
      return false;
    });

  EXPECT_EQ(
    navigator.start({{1.0, 1.0, 0.0}}),
    WaypointNavigatorStatus::FAILED);
  EXPECT_EQ(navigator.remaining(), 0u);
}

TEST(WaypointNavigator, InvalidWaypointFails)
{
  WaypointNavigator navigator(
    [](const Waypoint2D&) {
      return true;
    });

  Waypoint2D invalid;
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    navigator.start({invalid}), WaypointNavigatorStatus::FAILED);
}

TEST(WaypointNavigator, HaltCancelsAndClearsRoute)
{
  int cancelled = 0;
  WaypointNavigator navigator(
    [](const Waypoint2D&) {
      return true;
    },
    [&cancelled]() {
      ++cancelled;
    });

  EXPECT_EQ(
    navigator.start({{1.0, 1.0, 0.0}}),
    WaypointNavigatorStatus::RUNNING);
  navigator.halt();

  EXPECT_EQ(cancelled, 1);
  EXPECT_EQ(navigator.status(), WaypointNavigatorStatus::IDLE);
  EXPECT_EQ(navigator.remaining(), 0u);
}
