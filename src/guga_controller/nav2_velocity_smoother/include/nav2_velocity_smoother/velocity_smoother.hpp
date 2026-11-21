// Copyright (c) 2022 Samsung Research
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
#define NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "std_msgs/msg/bool.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_velocity_smoother
{

/**
 * @class nav2_velocity_smoother::VelocitySmoother
 * @brief This class that smooths cmd_vel velocities for robot bases
 */
class VelocitySmoother : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_velocity_smoother::VelocitySmoother
   * @param options Additional options to control creation of the node.
   */
  explicit VelocitySmoother(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for nav2_velocity_smoother::VelocitySmoother
   */
  ~VelocitySmoother();

  /**
   * @brief Find the scale factor, eta, which scales axis into acceleration range
   * @param v_curr current velocity
   * @param v_cmd commanded velocity
   * @param accel maximum acceleration
   * @param decel maximum deceleration
   * @return Scale factor, eta
   */
  double findEtaConstraint(
    const double v_curr, const double v_cmd,
    const double accel, const double decel);

  /**
   * @brief Apply acceleration and scale factor constraints
   * @param v_curr current velocity
   * @param v_cmd commanded velocity
   * @param accel maximum acceleration
   * @param decel maximum deceleration
   * @param eta Scale factor
   * @return Velocity command
   */
  double applyConstraints(
    const double v_curr, const double v_cmd,
    const double accel, const double decel, const double eta);

protected:
  /**
   * @brief Configures parameters and member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback for incoming velocity commands
   * @param msg Twist message
   */
  void inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Main worker timer function
   */
  void smootherTimer();

  /**
   * @brief Callback for the "hold still" switch from the decision tree
   * @param msg Bool message, true = force zero velocity
   */
  void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Whether the smoother should force zero velocity right now
   * @return True if the switch is on, or the heartbeat timed out
   */
  bool stopRequested();

  /**
   * @brief Dynamic reconfigure callback
   * @param parameters Parameter list to change
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Network interfaces
  std::unique_ptr<nav2_util::OdomSmoother> odom_smoother_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    smoothed_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Clock::SharedPtr clock_;
  geometry_msgs::msg::Twist last_cmd_;
  geometry_msgs::msg::Twist::SharedPtr command_;

  // Parameters
  double smoothing_frequency_;
  double odom_duration_;
  std::string odom_topic_;
  bool open_loop_;
  bool stopped_{true};
  bool scale_velocities_;
  std::vector<double> max_velocities_;
  std::vector<double> min_velocities_;
  std::vector<double> max_accels_;
  std::vector<double> max_decels_;
  std::vector<double> deadband_velocities_;
  rclcpp::Duration velocity_timeout_{0, 0};
  rclcpp::Time last_command_time_;

  // 决策树下发的"速度置零"开关。stop_timeout_ 为 0 表示不做心跳超时判断；
  // 大于 0 时，收到过消息之后若超过这个时间没有新消息，就按"要求停"处理。
  std::string stop_topic_;
  bool stop_{false};
  bool has_stop_msg_{false};
  rclcpp::Time stop_stamp_;
  rclcpp::Duration stop_timeout_{0, 0};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_velocity_smoother

#endif  // NAV2_VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
