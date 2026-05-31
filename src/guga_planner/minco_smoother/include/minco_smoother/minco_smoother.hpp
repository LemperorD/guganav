// Copyright 2025 LemperorD
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

#ifndef NAV2_SMOOTHER__MINCO_SMOOTHER_HPP_
#define NAV2_SMOOTHER__MINCO_SMOOTHER_HPP_

#include <nav2_core/smoother.hpp>
#include <minco_smoother/traj_representation.hpp>
#include <minco_smoother/minco.hpp>

#include <tf2/utils.hpp>
#include <minco_smoother/minco_utils.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace minco_smoother
{

/**
 * @struct Config
 * @brief Configuration kinematic constraints for MincoSmoother
 */
struct Config
{
    double max_vel_;
    double min_vel_;
    double max_acc_;
    double max_omega_;
    double max_domega_;
    double max_centripetal_acc_;

    bool if_directly_constrain_v_omega_;
};

/**
 * @class minco_smoother::minco_smoother
 * @brief use minco to optimize the path
 */
class MincoSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief Constructor for minco_smoother::minco_smoother
   */
  MincoSmoother() = default;

  /**
   * @brief Destructor for minco_smoother::minco_smoother
   */
  ~MincoSmoother() override = default;

  /**
   * @brief Configure smoother state machine
   * 
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap2d_sub Subscriber for costmap2d
   * @param footprint_sub Subscriber for footprint
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap2d_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  /**
   * @brief Cleanup smoother state machine
   */
  void cleanup() override;

  /**
   * @brief Activate smoother state machine
   */
  void activate() override;

  /**
   * @brief Deactivate smoother state machine
   */
  void deactivate() override;
  
  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be smoothed
   * @param max_time Maximum duration smoothing should take
   * @return If smoothing was completed (true) or interrupted by time limit (false)
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

protected:
  /**
   * @brief representation trajectory data from Nav2 Path to FlatTrajData
   * 
   * @param path Input path
   */
  FlatTrajData getTrajDataFromPath(const nav_msgs::msg::Path & path);

  bool minco_plan(FlatTrajData & flat_traj);

  /**
   * @brief get the optimized trajectory state
   * 
   * @param flat_traj Input flat trajectory data
   * @return true if get state successfully
   */
  bool get_state(const FlatTrajData & flat_traj);

  /**
   * @brief optimize the trajectory using L-BFGS
   * 
   * @return true if optimization is successful
   */
  bool optimizer();

  static inline int earlyExit(void *instance,
                              const Eigen::VectorXd &x,
                              const Eigen::VectorXd &g,
                              const double fx,
                              const double step,
                              const int k,
                              const int ls);

  static double costFunctionCallback(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g);

  // Gradient for partialGradByCoeffs and partialGradByTimes
  void attachPenaltyFunctional(double &cost);

  static double costFunctionCallbackPath(void *ptr,
                                         const Eigen::VectorXd &x,
                                         Eigen::VectorXd &g);

  void attachPenaltyFunctionalPath(double &cost);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoSmoother")};

  Eigen::Vector3d current_state_OAJ_;
  Eigen::Vector3d current_state_VAJ_;

  std::vector<Eigen::VectorXd> Unoccupied_sample_trajs_; // x y theta dtheta ds
  FlatTrajData flat_traj_;
  minco::Minco minco_;

  Eigen::Vector3d ICR_;
  bool if_standard_diff_;

  // Results
  Trajectory<7, 2> final_traj_;
  // Results before collision check
  Trajectory<7, 2> optimizer_traj_;
  // Results before trajectory pre-processing
  Trajectory<7, 2> init_final_traj_;

  Eigen::Vector3d start_pose_xytheta_;
  Eigen::Vector3d end_pose_xytheta_;
  nav_msgs::msg::Path smoothed_path_; //result path

  //---parameters---
  Config config_;
  double safe_dis_;
  double max_jps_dis_;
  double distance_weight_; double yaw_weight_;
  double sampletime_;
  int mintrajNum_;

  // optimizer parameters
  double mean_time_lowBound_;
  double mean_time_uppBound_;
  double smoothEps_;// for smoothL1
  PenaltyWeights penaltyWt_;
  Eigen::Vector2d energyWeights_;
  lbfgs::lbfgs_parameter_t lbfgs_params_;
  
  double finalMinSafeDis_;
  int finalSafeDisCheckNum_;
  int safeReplanMaxTime_;

  Eigen::Vector3d iniStateXYTheta_;
  Eigen::Vector3d finStateXYTheta_;

  Eigen::Vector3d final_initStateXYTheta_;
  Eigen::Vector3d final_finStateXYTheta_;

  Eigen::VectorXd pieceTime_;
  Eigen::MatrixXd Innerpoints_;
  Eigen::MatrixXd iniState_;
  Eigen::MatrixXd finState_;

  // trajectory segments number
  int TrajNum_;

  std::vector<Eigen::Vector3d> inner_init_positions_;

  Eigen::MatrixXd finalInnerpoints_;
  Eigen::VectorXd finalpieceTime_;

  std::vector<Eigen::Vector3d> statelist_;

  PathLbfgsParams path_lbfgs_params_;
  PathpenaltyWeights PathpenaltyWt_;

  // sampling parameters
  int sparseResolution_;
  int sparseResolution_6_;
  double timeResolution_;
  int mintrajNum_;

  int iter_num_;
  // store the gradient of the cost function
  Eigen::Matrix2Xd gradByPoints_;
  Eigen::VectorXd gradByTimes_;
  Eigen::MatrixX2d partialGradByCoeffs_;
  Eigen::VectorXd partialGradByTimes_;
  Eigen::Vector2d gradByTailStateS_;
  Eigen::Vector2d FinalIntegralXYError;
  // for ALM
  Eigen::Vector2d FinalIntegralXYError_;
  // for debug, record the collision points
  std::vector<Eigen::Vector2d> collision_point;
  std::vector<Eigen::Vector2d> collision_point_;

  // unchanged auxiliary parameters in the loop
  int SamNumEachPart_;
  // Simpson integration coefficients for each sampling point
  Eigen::VectorXd IntegralChainCoeff_;

  // checkpoints for collision check
  std::vector<Eigen::Vector2d> check_point;
  double safeDis_, safeDis;

  // Whether to perform visualization
  bool ifprint = false;

  // Augmented Lagrangian
  Eigen::VectorXd init_EqualLambda_, init_EqualRho_, EqualRhoMax_, EqualGamma_;
  Eigen::VectorXd EqualLambda, EqualRho;
  Eigen::VectorXd EqualTolerance_;

  bool hrz_limited_;
  double hrz_laser_range_dgr_;

  // Trajectory prediction resolution for get_the_predicted_state
  double trajPredictResolution_;

  bool if_visual_optimization_ = false;
};

} // namespace minco_smoother

#endif //NAV2_SMOOTHER__MINCO_SMOOTHER_HPP_