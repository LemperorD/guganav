#include "ROS2Monitor.hpp"

#include <memory>
#include <string>
#include <utility>

ROS2Monitor::ROS2Monitor(const rclcpp::NodeOptions& options)
: rclcpp::Node("ros2_monitor", options)
{
  // ===== 裁判数据订阅 =====
  // 用绝对话题名：实车 serial_driver_node 发布的是 "/referee/robot_status" 这类
  // 绝对名，不受命名空间影响。这里若写成相对名，策略节点一旦带着命名空间启动
  // （例如 /red_standard_robot1）就会解析成 /red_standard_robot1/referee/...，
  // 一条数据都收不到。
  const std::string topic =
    declare_parameter<std::string>("robot_status_topic", "/referee/robot_status");

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
      projectile_allowance_.store(msg->projectile_allowance_17mm);
      has_data_.store(true);
    });

  // ===== RFID 增益点订阅 =====
  // 同样用绝对名。实车由 serial_driver_node 从裁判系统 0x0209 解析后发布，
  // 机器人压到增益点的卡上时对应位才置 1，离开就回到 0。
  const std::string rfid_topic =
    declare_parameter<std::string>("rfid_status_topic", "/referee/rfid_status");
  sub_rfid_ = create_subscription<RfidStatusMsg>(
    rfid_topic, rclcpp::QoS(10),
    [this](RfidStatusMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      const bool base = msg->base_gain_point;
      const bool center = msg->center_gain_point;

      // 状态变化很少（一场比赛几次），按"变化"打日志比按时间节流更有用：
      // 复盘时能看到增益点是什么时候触发的。首条单独打，用来确认链路是通的。
      if (!has_rfid_data_.load()) {
        RCLCPP_INFO(get_logger(), "首次收到 RFID 状态：基地增益点=%d，中心增益点=%d",
                    base, center);
      } else if (base != base_gain_point_.load() ||
                 center != center_gain_point_.load()) {
        RCLCPP_INFO(get_logger(), "RFID 变化：基地增益点=%d，中心增益点=%d", base,
                    center);
      }

      base_gain_point_.store(base);
      center_gain_point_.store(center);
      has_rfid_data_.store(true);
    });

  // ===== 视觉订阅 =====
  // 话题名要与假数据源 / 真实视觉模块一致，做成参数便于带命名空间时调整。
  const std::string vision_topic =
    declare_parameter<std::string>("vision_topic", "vision/info");
  sub_vision_ = create_subscription<VisionInfo>(
    vision_topic, rclcpp::QoS(10),
    [this](VisionInfo::SharedPtr msg) {
      if (!msg) {
        return;
      }
      enemy_count_.store(msg->enemy_count);
    });

  RCLCPP_INFO(get_logger(), "ROS2Monitor 订阅话题: %s, %s, %s",
              topic.c_str(), rfid_topic.c_str(), vision_topic.c_str());

  // ===== 导航目标点发布 =====
  // 话题用相对名，实际话题由节点所在命名空间决定；frame_id 与项目其余部分一致为 map。
  const std::string goal_topic =
    declare_parameter<std::string>("goal_pose_topic", "goal_pose");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  // QoS 必须与 bt_navigator 的订阅一致：它用 rclcpp::SystemDefaultsQoS() 声明，
  // 在本机上实测解析成 BEST_EFFORT，与 rclcpp::SensorDataQoS() 相同。
  // 注意这不提供重传保证：订阅端是 best effort，发布端再"可靠"也不会收到确认，
  // 所以丢包要靠上层重发兜底（见 PublishGoal）。
  goal_pub_ = create_publisher<PoseStamped>(goal_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "目标点发布: 话题 %s，坐标系 %s",
              goal_topic.c_str(), frame_id_.c_str());

  // ===== 停车指令发布 =====
  // 绝对话题名：订阅方是底盘侧（serial_driver 一类，它用的是绝对名）。
  const std::string stop_topic =
    declare_parameter<std::string>("stop_topic", "/chassis_stop");
  stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic, rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "停车指令发布: 话题 %s（true = 要求停）", stop_topic.c_str());

  // ===== 定位：查 map -> base_frame_id 的变换 =====
  // 只建缓冲区，不在这里周期查询：查询放在 lookupRobotPose() 里按需进行，
  // 行为树每个 tick 查一次就够，比另起一个定时器少一处要维护的状态。
  //
  // TransformListener 传入本节点并开启独立线程：/tf 的订阅回调走它自己的
  // executor，与 main 里 tick 行为树的那条线程互不影响。
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true);

  // 底盘坐标系，与其余包里的 base_frame 保持一致。
  base_frame_id_ =
    declare_parameter<std::string>("base_frame_id", "base_footprint");

  RCLCPP_INFO(get_logger(), "定位查询: %s 在 %s 下的位置", base_frame_id_.c_str(),
              frame_id_.c_str());
}

void ROS2Monitor::publishStop(bool stop)
{
  std_msgs::msg::Bool msg;
  msg.data = stop;
  stop_pub_->publish(msg);

  // 只在变化时打日志：这是每 tick 都发的电平，按值打日志会刷屏。
  if (!has_last_stop_ || stop != last_stop_) {
    RCLCPP_INFO(get_logger(), "停车指令: %s", stop ? "要求停" : "解除");
    last_stop_ = stop;
    has_last_stop_ = true;
  }
}

bool ROS2Monitor::sendGoalPose(double x, double y, bool force)
{
  // Nav2 收到一次目标点就会开始导航，重复发同一个点会让它重启规划。
  if (!force && has_last_goal_ && x == last_goal_x_ && y == last_goal_y_) {
    return false;
  }

  PoseStamped goal;
  goal.header.frame_id = frame_id_;
  goal.header.stamp = now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  // 朝向必须是合法四元数：PoseStamped 的 orientation 默认全是 0，全 0 无法归一化，
  // 下游（TF、控制器）会得到 NaN，所以这里显式给单位四元数。
  // 注意单位四元数的含义是"目标朝向为 map 系 +x（yaw = 0）"，不是"不约束朝向"；
  // 本工程里朝向之所以不起作用，是因为控制器参数 yaw_goal_tolerance 设成了 6.28
  // （2π），任何朝向都算到位。要让到位朝向有意义，得先调小那个容差，再给目标加 yaw。
  goal.pose.orientation.w = 1.0;

  goal_pub_->publish(goal);

  last_goal_x_ = x;
  last_goal_y_ = y;
  has_last_goal_ = true;

  RCLCPP_INFO(get_logger(), "发布目标点 (%.2f, %.2f)，坐标系 %s",
              x, y, frame_id_.c_str());
  return true;
}

bool ROS2Monitor::lookupRobotPose(double& x, double& y)
{
  try {
    // 用 TimePointZero 而不是 now()：它表示"最近一条可用变换"。
    // 按 now() 查在仿真里很容易失败——TF 的时间戳来自仿真时钟，
    // 与查询时刻总有偏差，会周期性报 extrapolation。
    const auto tf =
      tf_buffer_->lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException& e) {
    // 定位没起来时每个 tick 都会走到这里，节流免得刷屏。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "查不到 %s 在 %s 下的位置，定位未就绪: %s",
                         base_frame_id_.c_str(), frame_id_.c_str(), e.what());
    return false;
  }
}
