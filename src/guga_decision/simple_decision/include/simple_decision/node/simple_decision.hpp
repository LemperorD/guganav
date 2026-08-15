#pragma once

#include <optional>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "guga_interfaces/msg/armors.hpp"
#include "guga_interfaces/msg/target.hpp"
#include "guga_interfaces/msg/game_status.hpp"
#include "guga_interfaces/msg/robot_status.hpp"

#include "simple_decision/adapter/transform.hpp"
#include "simple_decision/core/decision.hpp"
#include "simple_decision/core/environment_context.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace simple_decision {
  using RobotStatusMsg = guga_interfaces::msg::RobotStatus;
  using GameStatusMsg = guga_interfaces::msg::GameStatus;
  using ArmorsMsg = guga_interfaces::msg::Armors;
  using TargetMsg = guga_interfaces::msg::Target;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using UInt8Msg = std_msgs::msg::UInt8;
  using CmdSpinMsg = example_interfaces::msg::Float32;

  class DecisionSimple : public rclcpp::Node {
  public:
    explicit DecisionSimple(const rclcpp::NodeOptions& options);

  private:
    ContextConfig declareParams();
    void setupInfrastructure();

    void onGameStatus(GameStatusMsg::SharedPtr msg);
    void processDecision();
    [[nodiscard]] std::optional<Pose2D> getRobotPoseMap();
    void executeAction(DecisionAction action);
    void publishGoal(const PoseStampedMsg& goal, State state);
    void publishGoalThrottled(const PoseStampedMsg& goal,
                              rclcpp::Time& last_pub, double frequency);
    void publishChassisMode(ChassisMode mode);
    void publishCmdSpin(double speed);
    void handleGateLog(Readiness& readiness);

    [[nodiscard]] PoseStampedMsg makePoseXYZYaw(const std::string& frame,
                                                const Pose2D& position) const;
    static Stamp makeStamped(rclcpp::Time time);

    std::string frame_id_{"map"};
    std::string base_frame_id_{"base_link"};

    std::string robot_status_topic_{"referee/robot_status"};
    std::string goal_pose_topic_{"goal_pose"};
    std::string chassis_mode_topic_{"chassis_mode"};
    std::string debug_attack_pose_topic_{"debug_attack_pose"};
    std::string game_status_topic_{"referee/game_status"};
    std::string detector_armors_topic_{"detector/armors"};
    std::string tracker_target_topic_{"tracker/target"};
    std::string cmd_spin_topic_{"cmd_spin"};

    double tick_hz_{20.0};
    double default_goal_hz_{2.0};
    double supply_goal_hz_{2.0};
    double attack_goal_hz_{10.0};
    double start_delay_sec_{5.0};
    /// 启动即小陀螺：为 true 时无视决策输出的 chassis_mode，恒发 LITTLE_TES +
    /// cmd_spin
    bool always_tes_{true};
    /// 小陀螺自旋角速度（rad/s），经 cmd_spin 下发
    double spin_speed_{6.28};

    bool default_spin_latched_{false};

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<UInt8Msg>::SharedPtr chassis_mode_pub_;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr debug_attack_pose_pub_;
    rclcpp::Publisher<CmdSpinMsg>::SharedPtr cmd_spin_pub_;

    rclcpp::Subscription<RobotStatusMsg>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<GameStatusMsg>::SharedPtr game_status_sub_;
    rclcpp::Subscription<ArmorsMsg>::SharedPtr armors_sub_;
    rclcpp::Subscription<TargetMsg>::SharedPtr target_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Time last_default_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_supply_pub_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_attack_pub_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<EnvironmentContext> environment_;
    std::unique_ptr<Decision> controller_;

    friend class DecisionSimpleTest;
  };

}  // namespace simple_decision
