#pragma once

#include <cstddef>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace guga_decision_common
{

struct Waypoint2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum class WaypointNavigatorStatus
{
  IDLE,
  RUNNING,
  SUCCEEDED,
  FAILED,
};

// 与 ROS 和 BehaviorTree.CPP 都不绑定的顺序航点状态机。
//
// 调用方负责提供两个动作：
//   send_goal(waypoint)  发布/发送导航目标
//   cancel_goal()        中止当前导航目标
//
// 到达判断由调用方每个控制周期作为 update(arrived) 传入。这样同一套状态机
// 既可以配 goal_pose + TF 到达判断，也可以配 NavigateToPose action。
//
// 该类不是线程安全的，应在创建行为树和执行 tick 的同一线程中使用。
class WaypointNavigator
{
public:
  using SendGoal = std::function<bool(const Waypoint2D&)>;
  using CancelGoal = std::function<void()>;

  explicit WaypointNavigator(SendGoal send_goal,
                             CancelGoal cancel_goal = {});

  // 开始新路线。若已有路线正在执行，会先取消旧目标。
  // 空路线直接返回 SUCCEEDED。
  WaypointNavigatorStatus start(std::vector<Waypoint2D> route);

  // arrived 表示当前 currentWaypoint() 已经到达。
  // 到达后自动弹出当前点并发送下一个点；数组清空时返回 SUCCEEDED。
  WaypointNavigatorStatus update(bool arrived);

  // 中止当前路线、取消导航目标并清空路线。
  void halt();

  WaypointNavigatorStatus status() const { return status_; }
  bool running() const { return status_ == WaypointNavigatorStatus::RUNNING; }
  std::size_t remaining() const { return route_.size(); }
  std::size_t totalCount() const { return total_count_; }
  std::size_t completedCount() const { return completed_count_; }
  std::optional<Waypoint2D> currentWaypoint() const;

private:
  bool validWaypoint(const Waypoint2D& waypoint) const;
  bool sendCurrentGoal();
  void fail();
  void succeed();

  SendGoal send_goal_;
  CancelGoal cancel_goal_;
  std::deque<Waypoint2D> route_;
  WaypointNavigatorStatus status_{WaypointNavigatorStatus::IDLE};
  std::size_t total_count_{0};
  std::size_t completed_count_{0};
};

}  // namespace guga_decision_common
