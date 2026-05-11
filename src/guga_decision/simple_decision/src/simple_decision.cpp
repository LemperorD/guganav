#include "simple_decision/node/simple_decision.hpp"

namespace simple_decision {
  DecisionSimple::DecisionSimple(const rclcpp::NodeOptions& options)
      : Node("simple_decision", options) {
    const auto config = declareParams();
    environment_ = std::make_unique<EnvironmentContext>(config);
    controller_ = std::make_unique<Decision>(config);
    setupInfrastructure();

    RCLCPP_INFO(this->get_logger(),
                "simple_decision(min+mode) started. ns=%s goal_pose=%s "
                "robot_status=%s chassis_mode=%s",
                this->get_namespace(), goal_pose_topic_.c_str(),
                robot_status_topic_.c_str(), chassis_mode_topic_.c_str());
  }
  ContextConfig DecisionSimple::declareParams() {
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id",
                                                          "base_footprint");
    robot_status_topic_ = this->declare_parameter<std::string>(
        "referee_robot_status_topic", "referee/robot_status");
    goal_pose_topic_ = this->declare_parameter<std::string>("goal_pose_topic",
                                                            "goal_pose");
    chassis_mode_topic_ = this->declare_parameter<std::string>(
        "chassis_mode_topic", "chassis_mode");
    debug_attack_pose_topic_ = this->declare_parameter<std::string>(
        "debug_attack_pose_topic", "debug_attack_pose");
    game_status_topic_ = this->declare_parameter<std::string>(
        "referee_game_status_topic", "referee/game_status");
    detector_armors_topic_ = this->declare_parameter<std::string>(
        "detector_armors_topic", "detector/armors");
    tracker_target_topic_ = this->declare_parameter<std::string>(
        "tracker_target_topic", "tracker/target");
    tick_hz_ = this->declare_parameter<double>("tick_hz", 20.0);
    default_goal_hz_ = this->declare_parameter<double>("default_goal_hz", 2.0);
    supply_goal_hz_ = this->declare_parameter<double>("supply_goal_hz", 2.0);
    attack_goal_hz_ = this->declare_parameter<double>("attack_goal_hz", 10.0);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 5.0);

    const auto hp_enter = static_cast<int>(
        this->declare_parameter<int>("hp_survival_enter", 120));
    const auto hp_exit = static_cast<int>(
        this->declare_parameter<int>("hp_survival_exit", 300));
    const auto ammo_min = static_cast<int>(
        this->declare_parameter<int>("ammo_min", 0));
    const auto combat_max = this->declare_parameter<double>(
        "combat_max_distance", 8.0);
    const auto require_game = this->declare_parameter<bool>(
        "require_game_running", true);
    const auto default_x = this->declare_parameter<double>("default_x", 2.0);
    const auto default_y = this->declare_parameter<double>("default_y", 0.5);
    const auto default_yaw = this->declare_parameter<double>("default_yaw",
                                                             0.0);
    const auto supply_x = this->declare_parameter<double>("supply_x", 0.0);
    const auto supply_y = this->declare_parameter<double>("supply_y", 0.0);
    const auto supply_yaw = this->declare_parameter<double>("supply_yaw", 0.0);
    const auto arrive_tol = this->declare_parameter<double>(
        "default_arrive_xy_tol", 0.30);
    const auto spin_tol = this->declare_parameter<double>(
        "default_spin_keep_xy_tol", 0.80);

    return {hp_enter,         hp_exit,    ammo_min,   combat_max,  require_game,
            start_delay_sec_, default_x,  default_y,  default_yaw, supply_x,
            supply_y,         supply_yaw, arrive_tol, spin_tol};
  }
  void DecisionSimple::setupInfrastructure() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_pose_pub_ = this->create_publisher<PoseStampedMsg>(
        goal_pose_topic_, rclcpp::SensorDataQoS());
    chassis_mode_pub_ = this->create_publisher<UInt8Msg>(chassis_mode_topic_,
                                                         rclcpp::QoS(10));
    debug_attack_pose_pub_ = this->create_publisher<PoseStampedMsg>(
        debug_attack_pose_topic_, rclcpp::QoS(10));

    robot_status_sub_ = this->create_subscription<RobotStatusMsg>(
        robot_status_topic_, rclcpp::QoS(10),
        [this](RobotStatusMsg::SharedPtr msg) {
          environment_->onRobotStatus(ConvertRobotStatus(std::move(msg)));
        });
    game_status_sub_ = this->create_subscription<GameStatusMsg>(
        game_status_topic_, rclcpp::QoS(10),
        [this](GameStatusMsg::SharedPtr msg) { onGameStatus(std::move(msg)); });
    armors_sub_ = this->create_subscription<ArmorsMsg>(
        detector_armors_topic_, rclcpp::QoS(10),
        [this](ArmorsMsg::SharedPtr msg) {
          environment_->onArmors(ConvertArmors(msg));
        });
    target_sub_ = this->create_subscription<TargetMsg>(
        tracker_target_topic_, rclcpp::QoS(10),
        [this](TargetMsg::SharedPtr msg) {
          environment_->onTarget(ConvertTarget(msg));
        });

    const double frequency = std::max(1e-6, tick_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / frequency));
    timer_ = this->create_wall_timer(period, [this] { processDecision(); });
  }

  void DecisionSimple::onGameStatus(const GameStatusMsg::SharedPtr msg) {
    const auto now = this->now();
    environment_->onGameStatus(ConvertGameStatus(msg),
                               static_cast<int64_t>(now.nanoseconds()));

    if (environment_->isGameStarted()) {
      RCLCPP_INFO(this->get_logger(),
                  "Game started detected, delaying decision for %.1f seconds",
                  start_delay_sec_);
    }

    if (environment_->isGameOver()) {
      default_spin_latched_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Game left running state, reset match-start gate.");
      environment_->resetGameOver();
    }
  }

  void DecisionSimple::processDecision() {
    const auto now = this->now();
    auto readiness = environment_->checkReadiness(now.nanoseconds());

    if (readiness.status != Readiness::Status::READY) {
      handleGateLog(readiness);
      return;
    }
    if (auto opt_pose = getRobotPoseMap()) {
      environment_->updatePose(opt_pose->x, opt_pose->y, opt_pose->yaw);
    }

    Snapshot snapshot = environment_->getSnapshot(makeStamped(now));
    auto action = controller_->computeAction(snapshot);
    executeAction(action);
  }

  Stamp DecisionSimple::makeStamped(rclcpp::Time time) {
    Stamp stamp{static_cast<int32_t>(time.seconds()),
                static_cast<uint32_t>(time.nanoseconds() % 1'000'000'000UL)};
    return stamp;
  }

  void DecisionSimple::executeAction(DecisionAction action) {
    if (environment_->changeState(action.next_state)) {
      RCLCPP_INFO(this->get_logger(), "State -> %u",
                  static_cast<unsigned>(action.next_state));
    }
    publishChassisMode(action.chassis_mode);

    if (action.should_publish_goal) {
      const auto goal = makePoseXYZYaw(
          frame_id_, {action.target_x, action.target_y, action.target_yaw});
      publishGoal(goal, action.next_state);
      if (action.next_state == State::ATTACK) {
        debug_attack_pose_pub_->publish(goal);
      }
    }
  }

  void DecisionSimple::publishGoal(const PoseStampedMsg& goal, State state) {
    if (state == State::SUPPLY) {
      publishGoalThrottled(goal, last_supply_pub_, supply_goal_hz_);
    } else if (state == State::ATTACK) {
      publishGoalThrottled(goal, last_attack_pub_, attack_goal_hz_);
    } else {
      publishGoalThrottled(goal, last_default_pub_, default_goal_hz_);
    }
  }

  PoseStampedMsg DecisionSimple::makePoseXYZYaw(const std::string& frame,
                                                const Pose2D& position) const {
    PoseStampedMsg p;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, position.yaw);

    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.header.frame_id = frame;
    p.header.stamp = this->now();
    p.pose.position.x = position.x;
    p.pose.position.y = position.y;
    p.pose.position.z = 0.0;

    return p;
  }

  std::optional<Pose2D> DecisionSimple::getRobotPoseMap() {
    try {
      const auto tf = tf_buffer_->lookupTransform(frame_id_, base_frame_id_,
                                                  tf2::TimePointZero);
      Pose2D pose{};
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      const double qx = tf.transform.rotation.x;
      const double qy = tf.transform.rotation.y;
      const double qz = tf.transform.rotation.z;
      const double qw = tf.transform.rotation.w;
      pose.yaw = std::atan2(2.0 * ((qw * qz) + (qx * qy)),
                            1.0 - (2.0 * ((qy * qy) + (qz * qz))));
      return pose;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "TF lookup failed (%s -> %s): %s",
                            frame_id_.c_str(), base_frame_id_.c_str(),
                            ex.what());
      return std::nullopt;
    }
  }

  void DecisionSimple::publishChassisMode(ChassisMode mode) {
    UInt8Msg m;
    m.data = static_cast<uint8_t>(mode);
    chassis_mode_pub_->publish(m);
  }

  void DecisionSimple::publishGoalThrottled(const PoseStampedMsg& goal,
                                            rclcpp::Time& last_pub,
                                            double frequency) {
    const double period = (frequency <= 1e-6) ? 1e9 : (1.0 / frequency);
    const auto now = this->now();

    if (last_pub.nanoseconds() == 0 || (now - last_pub).seconds() >= period) {
      last_pub = now;
      goal_pose_pub_->publish(goal);
    }
  }

  void DecisionSimple::handleGateLog(Readiness& readiness) {
    using Status = Readiness::Status;
    switch (readiness.status) {
      case Status::NO_RS:
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Gate: no robot_status received; withholding decisions");
        break;
      case Status::NO_GS:
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Gate: no game_status received; waiting for referee");
        break;
      case Status::NOT_STARTED:
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Gate: match not started yet; decisions disabled");
        break;
      case Status::IN_DELAY:
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Gate: match started but delaying decisions (elapsed=%.2f s)",
            readiness.elapsed);
        break;
      default:
        break;
    }
  }

}  // namespace simple_decision

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_decision::DecisionSimple)