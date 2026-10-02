#include "fake_msg_source/fake_msg_source.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
// RMUL 哨兵满状态（U26 表 3-4）
constexpr std::int64_t kSentryId = 7;
constexpr std::int64_t kSentryMaxHp = 400;
constexpr std::int64_t kSentryHeatLimit = 260;
constexpr std::int64_t kSentryCooling = 30;

constexpr double kDefaultStatusRate = 20.0;
constexpr double kDefaultVisionRate = 10.0;
}  // namespace

FakeMsgSource::FakeMsgSource()
: rclcpp::Node("fake_msg_source")
{
  // ===== 话题与频率 =====
  // 裁判两路用绝对名，与实车 serial_driver_node 发布的名称一致（它写的是
  // "/referee/robot_status" 这种绝对名，不受命名空间影响）。
  const std::string status_topic =
    declare_parameter<std::string>("robot_status_topic", "/referee/robot_status");
  const std::string rfid_topic =
    declare_parameter<std::string>("rfid_status_topic", "/referee/rfid_status");
  const std::string vision_topic =
    declare_parameter<std::string>("vision_topic", "vision/info");
  const double status_rate = declare_parameter<double>("publish_rate", kDefaultStatusRate);
  const double vision_rate = declare_parameter<double>("vision_rate", kDefaultVisionRate);

  // ===== 本机裁判状态：全部可在运行时调整 =====
  declare_parameter<std::int64_t>("robot_id", kSentryId);
  declare_parameter<std::int64_t>("robot_level", 1);
  declare_parameter<std::int64_t>("current_hp", kSentryMaxHp);
  declare_parameter<std::int64_t>("maximum_hp", kSentryMaxHp);
  declare_parameter<std::int64_t>("shooter_barrel_cooling_value", kSentryCooling);
  declare_parameter<std::int64_t>("shooter_barrel_heat_limit", kSentryHeatLimit);
  declare_parameter<std::int64_t>("shooter_17mm_1_barrel_heat", 0);
  declare_parameter<std::int64_t>("projectile_allowance_17mm", 100);
  declare_parameter<std::int64_t>("remaining_gold_coin", 0);

  // ===== RFID 增益点 =====
  // 只模拟本工程用到的两处：基地增益点（"家"）与中心增益点（RMUL 占点）。
  // 其余 22 位保持 false；以后要用别的点，在这里加一个 bool 参数即可。
  // 真机上这两位的来源是裁判系统 0x0209 的位域，机器人压到卡上才会置 1。
  declare_parameter<bool>("rfid_base", false);
  declare_parameter<bool>("rfid_center", false);

  // ===== 视觉汇总 =====
  // 只有一个计数字段：> 0 表示视野里有敌人。改成 0 即可模拟敌人消失。
  declare_parameter<std::int64_t>("enemy_count", 0);
  declare_parameter<std::string>("vision_frame_id", "map");

  // ===== 假位姿 =====
  // 决策判断"是否到达"用的是 map -> base_footprint，所以这里要发两段变换：
  //   odom -> base_footprint  机器人位置，每帧更新（对应实车的里程计）
  //   map -> odom             静态单位变换（对应实车的定位输出，让 odom 与 map 重合）
  // 于是 odom 系的坐标数值与 map 系相同，参数里写的就是 map 系位置。
  const std::string odom_topic =
    declare_parameter<std::string>("odom_topic", "odometry");
  const std::string goal_pose_topic =
    declare_parameter<std::string>("goal_pose_topic", "goal_pose");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_footprint");
  // 初始位置。速度非 0 时这两个参数会随机器人移动被写回，面板上能直接看到位置。
  declare_parameter<double>("robot_x", 2.0);
  declare_parameter<double>("robot_y", 0.0);
  declare_parameter<double>("robot_yaw", 0.0);
  // 0 表示机器人不动：位置完全由上面的参数决定，用来手试"未到达/已到达"两条分支。
  // 默认取小值是为了配合可视化：界面每完成一次 tick 才刷新一次光条，机器人一个
  // tick 走的距离必须明显小于巡逻方框的边长（0.5 m），否则看到的路径总是落后的。
  declare_parameter<double>("sim_speed", 0.2);
  declare_parameter<bool>("publish_map_to_odom", true);

  param_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      return on_parameters_changed(params);
    });

  status_pub_ = create_publisher<RobotStatusMsg>(status_topic, 10);
  rfid_pub_ = create_publisher<RfidStatusMsg>(rfid_topic, 10);
  vision_pub_ = create_publisher<VisionInfoMsg>(vision_topic, 10);
  odom_pub_ = create_publisher<OdometryMsg>(odom_topic, 10);

  // 订阅决策节点发出的目标点，用来模拟机器人朝目标移动。QoS 必须与发布端匹配：
  // 决策节点用的是 SensorDataQoS（best effort），订阅端写可靠就一个字节都收不到。
  goal_sub_ = create_subscription<PoseStampedMsg>(
    goal_pose_topic, rclcpp::SensorDataQoS(),
    [this](PoseStampedMsg::SharedPtr msg) {
      if (!msg) {
        return;
      }
      goal_x_ = msg->pose.position.x;
      goal_y_ = msg->pose.position.y;
      has_goal_ = true;
    });

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  last_hp_ = static_cast<std::uint16_t>(param_int("current_hp"));

  // 频率非法时回退到默认值，避免除零或负周期
  const double eff_status_rate = (status_rate > 0.0) ? status_rate : kDefaultStatusRate;
  const double eff_vision_rate = (vision_rate > 0.0) ? vision_rate : kDefaultVisionRate;

  // map -> odom 是静态的，发一次就够（latched）。
  if (param_bool("publish_map_to_odom")) {
    geometry_msgs::msg::TransformStamped tf_map_odom;
    tf_map_odom.header.stamp = now();
    tf_map_odom.header.frame_id = param_str("map_frame");
    tf_map_odom.child_frame_id = param_str("odom_frame");
    tf_map_odom.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf_map_odom);
  }

  // 裁判两路同频：真机上它们来自同一帧串口数据，分开两个频率没有意义。
  // 位姿也跟着这个频率走，决策节点每个 tick 读到的位置就是这里积分的结果。
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / eff_status_rate),
    [this, eff_status_rate]() {
      step_pose(1.0 / eff_status_rate);
      publish_robot_status();
      publish_rfid_status();
      publish_pose();
    });
  vision_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / eff_vision_rate),
    [this]() { publish_vision_info(); });

  RCLCPP_INFO(
    get_logger(),
    "假数据源已启动：%s @ %.1f Hz，%s @ %.1f Hz，%s @ %.1f Hz，%s @ %.1f Hz，ID %ld，"
    "HP %u/%ld，enemy_count %ld，RFID 基地=%d 中心=%d",
    status_topic.c_str(), eff_status_rate, rfid_topic.c_str(), eff_status_rate,
    vision_topic.c_str(), eff_vision_rate, odom_topic.c_str(), eff_status_rate,
    param_int("robot_id"), last_hp_, param_int("maximum_hp"),
    param_int("enemy_count"), param_bool("rfid_base"), param_bool("rfid_center"));
  RCLCPP_INFO(
    get_logger(),
    "假位姿：%s -> %s，%s -> %s（%s），起点 (%.2f, %.2f)，速度 %.2f m/s",
    param_str("odom_frame").c_str(), param_str("base_frame").c_str(),
    param_str("map_frame").c_str(), param_str("odom_frame").c_str(),
    param_bool("publish_map_to_odom") ? "静态单位变换" : "不再发布，由真实定位负责",
    param_double("robot_x"), param_double("robot_y"), param_double("sim_speed"));
}

rcl_interfaces::msg::SetParametersResult FakeMsgSource::on_parameters_changed(
  const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 参数值本身由 ROS 2 保存，这里只回报改了哪些，便于确认调整是否生效。
  // robot_x/robot_y/robot_yaw 例外：机器人移动时每帧都会写回这三个参数，打日志
  // 会刷屏；位置在面板上直接看就行。
  for (const auto& param : params) {
    const std::string& name = param.get_name();
    if (name == "robot_x" || name == "robot_y" || name == "robot_yaw") {
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "参数 %s 改为 %s",
      name.c_str(), param.value_to_string().c_str());
  }
  return result;
}

void FakeMsgSource::step_pose(double dt)
{
  const double speed = param_double("sim_speed");
  if (speed <= 0.0 || !has_goal_) {
    // 速度 0：机器人钉在参数给的位置上，用来手试"未到达/已到达"两条分支。
    return;
  }

  double x = param_double("robot_x");
  double y = param_double("robot_y");
  const double dx = goal_x_ - x;
  const double dy = goal_y_ - y;
  const double distance = std::hypot(dx, dy);
  const double step = speed * dt;

  if (distance <= step) {
    // 一步能跨过就直接落到目标点上，免得在目标点附近来回抖。
    x = goal_x_;
    y = goal_y_;
  } else {
    x += dx / distance * step;
    y += dy / distance * step;
  }

  set_parameters({
    rclcpp::Parameter("robot_x", x),
    rclcpp::Parameter("robot_y", y),
    rclcpp::Parameter("robot_yaw", std::atan2(dy, dx)),
  });
}

void FakeMsgSource::publish_pose()
{
  const double x = param_double("robot_x");
  const double y = param_double("robot_y");
  const double yaw = param_double("robot_yaw");

  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);

  const auto stamp = now();
  const std::string odom_frame = param_str("odom_frame");
  const std::string base_frame = param_str("base_frame");

  OdometryMsg odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation = q;
  // 速度按"朝目标走的实际速度"填：没目标或速度为 0 时就是 0。
  const double speed = param_double("sim_speed");
  if (has_goal_ && speed > 0.0) {
    const double dx = goal_x_ - x;
    const double dy = goal_y_ - y;
    const double distance = std::hypot(dx, dy);
    if (distance > 1e-6) {
      odom.twist.twist.linear.x = dx / distance * speed;
      odom.twist.twist.linear.y = dy / distance * speed;
    }
  }
  odom_pub_->publish(odom);

  // 决策节点查的就是这段变换：map -> base_footprint 由它和静态的 map -> odom 拼出来。
  geometry_msgs::msg::TransformStamped tf_odom_base;
  tf_odom_base.header.stamp = stamp;
  tf_odom_base.header.frame_id = odom_frame;
  tf_odom_base.child_frame_id = base_frame;
  tf_odom_base.transform.translation.x = x;
  tf_odom_base.transform.translation.y = y;
  tf_odom_base.transform.rotation = q;
  tf_broadcaster_->sendTransform(tf_odom_base);
}

void FakeMsgSource::publish_robot_status()
{
  RobotStatusMsg msg;

  msg.robot_id = static_cast<std::uint8_t>(param_int("robot_id"));
  msg.robot_level = static_cast<std::uint8_t>(param_int("robot_level"));
  msg.current_hp = static_cast<std::uint16_t>(param_int("current_hp"));
  msg.maximum_hp = static_cast<std::uint16_t>(param_int("maximum_hp"));
  msg.shooter_barrel_cooling_value =
    static_cast<std::uint16_t>(param_int("shooter_barrel_cooling_value"));
  msg.shooter_barrel_heat_limit =
    static_cast<std::uint16_t>(param_int("shooter_barrel_heat_limit"));
  msg.shooter_17mm_1_barrel_heat =
    static_cast<std::uint16_t>(param_int("shooter_17mm_1_barrel_heat"));
  msg.projectile_allowance_17mm =
    static_cast<std::uint16_t>(param_int("projectile_allowance_17mm"));
  msg.remaining_gold_coin = static_cast<std::uint16_t>(param_int("remaining_gold_coin"));

  // robot_pos 留原点：RMUL 下哨兵位置由定位给出，裁判系统不下发。
  // armor_id 与 hp_deduction_reason 留 0：这里不模拟具体扣血事件。

  // is_hp_deduced 是上位机二次处理的标志，按相邻两帧比较得出。
  msg.is_hp_deduced = (msg.current_hp < last_hp_);
  last_hp_ = msg.current_hp;

  status_pub_->publish(msg);
}

void FakeMsgSource::publish_rfid_status()
{
  RfidStatusMsg msg;

  // 只填本工程用到的两位，其余保持 false：真机上未触发的位也是 0。
  msg.base_gain_point = param_bool("rfid_base");
  msg.center_gain_point = param_bool("rfid_center");

  rfid_pub_->publish(msg);
}

void FakeMsgSource::publish_vision_info()
{
  VisionInfoMsg msg;
  msg.header.stamp = now();
  msg.header.frame_id = get_parameter("vision_frame_id").as_string();
  msg.enemy_count = static_cast<std::int32_t>(param_int("enemy_count"));

  vision_pub_->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeMsgSource>());
  rclcpp::shutdown();
  return 0;
}
