#include "ROS2Monitor.hpp"

#include <memory>
#include <string>
#include <utility>

ROS2Monitor::ROS2Monitor(const rclcpp::NodeOptions& options)
: rclcpp::Node("ros2_monitor", options)
{
  // ===== 裁判数据订阅 =====
  const std::string topic =
    declare_parameter<std::string>("robot_status_topic", "referee/robot_status");

  // 回调在 executor 线程执行，tick() 在另一条线程执行，
  // 所以共享的成员一律用 std::atomic，回调里不碰黑板也不做判断。
  // 订阅必须由成员持有：存成局部变量的话，构造函数返回时 shared_ptr 析构，
  // 订阅随之销毁，话题上就再也看不到它，回调也不会被调用。
  sub_state_ = create_subscription<RobotStatusMsg>(
    topic, rclcpp::QoS(10),
    [this](RobotStatusMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      current_hp_.store(msg->current_hp);
      maximum_hp_.store(msg->maximum_hp);
      has_data_.store(true);
    });

  RCLCPP_INFO(get_logger(), "ROS2Monitor 订阅话题: %s", topic.c_str());

  // ===== 导航目标点发布 =====
  // 话题用相对名，实际话题由节点所在命名空间决定；frame_id 与项目其余部分一致为 map。
  const std::string goal_topic =
    declare_parameter<std::string>("goal_pose_topic", "goal_pose");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  // QoS 必须与 bt_navigator 的订阅一致，它用的是 BEST_EFFORT，
  // rclcpp::SensorDataQoS() 正好是 best effort。
  goal_pub_ = create_publisher<PoseStamped>(goal_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "目标点发布: 话题 %s，坐标系 %s",
              goal_topic.c_str(), frame_id_.c_str());
}

bool ROS2Monitor::sendGoalPose(double x, double y)
{
  // Nav2 收到一次目标点就会开始导航，重复发同一个点会让它重启规划。
  if (has_last_goal_ && x == last_goal_x_ && y == last_goal_y_) {
    return false;
  }

  PoseStamped goal;
  goal.header.frame_id = frame_id_;
  goal.header.stamp = now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  // 只给位置不约束朝向，用单位四元数。
  goal.pose.orientation.w = 1.0;

  goal_pub_->publish(goal);

  last_goal_x_ = x;
  last_goal_y_ = y;
  has_last_goal_ = true;

  RCLCPP_INFO(get_logger(), "发布目标点 (%.2f, %.2f)，坐标系 %s",
              x, y, frame_id_.c_str());
  return true;
}
