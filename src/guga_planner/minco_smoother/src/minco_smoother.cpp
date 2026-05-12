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

#include "minco_smoother/minco_smoother.hpp"
#include "minco_smoother/lbfgs.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

namespace minco_smoother
{

using nav2_util::declare_parameter_if_not_declared;

void MincoSmoother::cleanup(){
  costmap_sub_.reset();
  RCLCPP_INFO(logger_, "Cleaning up minco_ Smoother");
}

void MincoSmoother::activate(){
  RCLCPP_INFO(logger_, "Activating minco_ Smoother");
}

void MincoSmoother::deactivate(){
  RCLCPP_INFO(logger_, "Deactivating minco_ Smoother");
}

void MincoSmoother::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                              std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
                              std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub){
  auto node = parent.lock();
  node_ = parent;
  if (!node) { throw nav2_core::PlannerException("Unable to lock node!"); }
  logger_ = node->get_logger();

  costmap_sub_ = costmap_sub; tf_ = tf;
  plugin_name_ = name; logger_ = node->get_logger();

  current_state_VAJ_ << 0.0, 0.0, 0.0;
  current_state_OAJ_ << 0.0, 0.0, 0.0;

  //---declare parameters---
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_velocity", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_acc", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_omega", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_domega", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".safe_distance", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_jps_dis", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".distance_weight", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_weight", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_traj_num", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sample_time", rclcpp::ParameterValue(0.9));
    
  //---get parameters---
  node->get_parameter(plugin_name_ + ".max_velocity", config_.max_vel_);
  node->get_parameter(plugin_name_ + ".max_acc", config_.max_acc_);
  node->get_parameter(plugin_name_ + ".max_omega", config_.max_omega_);
  node->get_parameter(plugin_name_ + ".max_domega", config_.max_domega_);
  node->get_parameter(plugin_name_ + ".safe_distance", safe_dis_);
  node->get_parameter(plugin_name_ + ".max_jps_dis", max_jps_dis_);
  node->get_parameter(plugin_name_ + ".distance_weight", distance_weight_);
  node->get_parameter(plugin_name_ + ".yaw_weight", yaw_weight_);
  node->get_parameter(plugin_name_ + ".min_traj_num", mintrajNum_);
  node->get_parameter(plugin_name_ + ".sample_time", sampletime_);

  SamNumEachPart_ = 2 * sparseResolution_;
  sparseResolution_6_ = sparseResolution_ * 6;
  IntegralChainCoeff_.resize(SamNumEachPart_ + 1);
  IntegralChainCoeff_.setZero();
  for(int i=0; i<sparseResolution_; i++){
    IntegralChainCoeff_.block(2*i,0,3,1) += Eigen::Vector3d(1.0, 4.0, 1.0);
  }
}

bool MincoSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time){
  if (path.poses.size() < 2) {
    RCLCPP_WARN(logger_, "Path too short to smooth");
    return false;
  }

  start_pose_xytheta_ << path.poses.front().pose.position.x,
                         path.poses.front().pose.position.y,
                         tf2::getYaw(path.poses.front().pose.orientation);
  end_pose_xytheta_ << path.poses.back().pose.position.x,
                       path.poses.back().pose.position.y,
                       tf2::getYaw(path.poses.back().pose.orientation);
  flat_traj_ = getTrajDataFromPath(path);

  if (!minco_plan(flat_traj_)) {
    RCLCPP_WARN(logger_, "MINCO smoother failed");
    return false;
  }

  smoothed_path_.header = path.header;

  double t = 0.0; double dt = sampletime_;

  while (t <= optimizer_traj_.getTotalDuration()) {
    Eigen::Vector2d pos = optimizer_traj_.getPos(t);
    Eigen::Vector2d vel = optimizer_traj_.getVel(t);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();

    double yaw = std::atan2(vel.y(), vel.x());
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    smoothed_path_.poses.push_back(pose);
    t += dt;
  }

  path = smoothed_path_;
  return true;
}

FlatTrajData MincoSmoother::getTrajDataFromPath(const nav_msgs::msg::Path & path){
  //-------getSamplePoints---------
  Unoccupied_sample_trajs_.clear();
  double cur_theta;
  Eigen::VectorXd state5d; state5d.resize(5);// x y theta dtheta ds

  state5d << path.poses[0].pose.position.x, path.poses[0].pose.position.y, 0, 0, 0;
  Unoccupied_sample_trajs_.push_back(state5d); 

  cur_theta = atan2(path.poses[1].pose.position.y - path.poses[0].pose.position.y, path.poses[1].pose.position.x - path.poses[0].pose.position.x);
  normalizeAngle(0, cur_theta);
  state5d << path.poses[0].pose.position.x, path.poses[0].pose.position.y, cur_theta , cur_theta, 0;
  Unoccupied_sample_trajs_.push_back(state5d);

  cur_theta = atan2(path.poses[0].pose.position.y - path.poses[1].pose.position.y, path.poses[0].pose.position.x - path.poses[1].pose.position.x) + M_PI;
  normalizeAngle(0, cur_theta);
  state5d << path.poses[0].pose.position.x, path.poses[0].pose.position.y, cur_theta , cur_theta, 0;
  Unoccupied_sample_trajs_.push_back(state5d); // 2
    
  int path_size = path.poses.size();
  geometry_msgs::msg::PoseStamped pt;
  for(int i = 1; i<path_size-1; i++){ 
    pt = path.poses[i];

    state5d << pt.pose.position.x, pt.pose.position.y, Unoccupied_sample_trajs_.back()[2], 0, 
               hypot(pt.pose.position.x - Unoccupied_sample_trajs_.back()[0], pt.pose.position.y - Unoccupied_sample_trajs_.back()[1]);
    Unoccupied_sample_trajs_.push_back(state5d);
    cur_theta = atan2(path.poses[i+1].pose.position.y - path.poses[i].pose.position.y, 
                      path.poses[i].pose.position.x - path.poses[i].pose.position.x);
    normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
    state5d << pt.pose.position.x, pt.pose.position.y, cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
    Unoccupied_sample_trajs_.push_back(state5d);
  }

  pt = path.poses.back();
  state5d << pt.pose.position.x, pt.pose.position.y, Unoccupied_sample_trajs_.back()[2], 0, 
             hypot(pt.pose.position.x - Unoccupied_sample_trajs_.back()[0], pt.pose.position.y - Unoccupied_sample_trajs_.back()[1]);
  Unoccupied_sample_trajs_.push_back(state5d);

  cur_theta = tf2::getYaw(pt.pose.orientation);
  normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
  state5d << pt.pose.position.x, pt.pose.position.y, cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
  Unoccupied_sample_trajs_.push_back(state5d);

  //-------getTrajsWithTime(no cut)---------
  std::vector<double> Unoccupied_thetas;
  std::vector<double> Unoccupied_pathlengths; 
  std::vector<double> Unoccupied_Weightpathlengths; 

  double Unoccupied_AllWeightingPathLength_ = 0; 
  double Unoccupied_AllPathLength = 0;

  int PathNodeNum = Unoccupied_sample_trajs_.size();
  Unoccupied_thetas.push_back(Unoccupied_sample_trajs_[0][2]);
  Unoccupied_pathlengths.push_back(0);    
  Unoccupied_Weightpathlengths.push_back(0);

  int pathnodeindex = 1;
  for(; pathnodeindex<PathNodeNum; pathnodeindex++){
    Eigen::VectorXd pathnode = Unoccupied_sample_trajs_[pathnodeindex];
    Unoccupied_thetas.push_back(pathnode[2]);
    Unoccupied_AllPathLength += pathnode[4];
    Unoccupied_pathlengths.push_back(Unoccupied_AllPathLength); 
    Unoccupied_AllWeightingPathLength_ += yaw_weight_ * abs(pathnode[3]) + distance_weight_ * abs(pathnode[4]);
    Unoccupied_Weightpathlengths.push_back(Unoccupied_AllWeightingPathLength_);
  }

  double totalTrajTime_ = evaluateDuration(Unoccupied_AllWeightingPathLength_, current_state_VAJ_.x(), 0.0,
                                           config_.max_vel_, config_.max_acc_);
  std::vector<Eigen::Vector3d> Unoccupied_traj_pts; // Store the sampled coordinates yaw, s, t
  std::vector<Eigen::Vector3d> Unoccupied_positions; // Store the sampled coordinates x, y, yaw

  double Unoccupied_totalTrajTime_ = totalTrajTime_;
  double Unoccupied_sampletime;
  int Unoccupied_PathNodeIndex = 1;

  Unoccupied_sampletime = Unoccupied_totalTrajTime_ / std::max(int(Unoccupied_totalTrajTime_ / sampletime_ + 0.5), mintrajNum_);

  double tmparc = 0;

  for(double samplet = Unoccupied_sampletime; samplet<Unoccupied_totalTrajTime_-1e-3; samplet+=Unoccupied_sampletime){
    double arc = evaluateLength(samplet, Unoccupied_AllWeightingPathLength_,
                                Unoccupied_totalTrajTime_, current_state_VAJ_.x(), 0.0, config_.max_vel_, config_.max_acc_);
    for (int k = Unoccupied_PathNodeIndex; k<PathNodeNum; k++){
      Eigen::VectorXd pathnode = Unoccupied_sample_trajs_[k];
      Eigen::VectorXd prepathnode = Unoccupied_sample_trajs_[k-1];
      tmparc = Unoccupied_Weightpathlengths[k];
      if(tmparc >= arc){  
        Unoccupied_PathNodeIndex = k; 
        double l1 = tmparc-arc;
        double l = Unoccupied_Weightpathlengths[k]-Unoccupied_Weightpathlengths[k-1];
        double interp_s = Unoccupied_pathlengths[k-1] + (l-l1)/l*(pathnode[4]);
        double interp_yaw = Unoccupied_sample_trajs_[k-1][2] + (l-l1)/l*(pathnode[3]);
        Unoccupied_traj_pts.emplace_back(interp_yaw, interp_s, samplet);

        double interp_x = l1/l*prepathnode[0] + (l-l1)/l*(pathnode[0]);
        double interp_y = l1/l*prepathnode[1] + (l-l1)/l*(pathnode[1]);
        Unoccupied_positions.emplace_back(interp_x, interp_y, interp_yaw);
        break;
      }
    }
  }

  Eigen::MatrixXd startS;
  Eigen::MatrixXd endS;
  startS.resize(2,3);
  endS.resize(2,3);  
  Eigen::Vector2d startP(Unoccupied_sample_trajs_[0][2],0);
  Eigen::Vector2d finalP(Unoccupied_sample_trajs_[PathNodeNum-1][2],Unoccupied_pathlengths[PathNodeNum-1]);
  startS.col(0) = startP;
  startS.block(0,1,1,2) = current_state_OAJ_.transpose().head(2);
  startS.block(1,1,1,2) = current_state_VAJ_.transpose().head(2);
  endS.col(0) = finalP;
  endS.col(1).setZero();
  endS.col(2).setZero();
  flat_traj_.UnOccupied_traj_pts = Unoccupied_traj_pts;
  flat_traj_.UnOccupied_initT = Unoccupied_sampletime;
  flat_traj_.UnOccupied_positions = Unoccupied_positions;
  
  flat_traj_.start_state = startS;
  flat_traj_.final_state = endS;
  flat_traj_.start_state_XYTheta = start_pose_xytheta_;
  flat_traj_.final_state_XYTheta = end_pose_xytheta_;;
    
  return flat_traj_;
}

bool MincoSmoother::minco_plan(FlatTrajData & flat_traj){
  bool final_collision = false;
  int replan_num_for_coll = 0;

  double start_safe_dis = map_->getDistanceReal(flat_traj.start_state_XYTheta.head(2))*0.85;
  safeDis = std::min(start_safe_dis, safeDis_);
  
  for(; replan_num_for_coll < safeReplanMaxTime; replan_num_for_coll++){
    if(get_state(flat_traj))
        RCLCPP_INFO(logger_, "\033[40;36m get_state time:%f \033[0m", (ros::Time::now()-current).toSec());
    else
        return false;
    current = ros::Time::now();
    if(optimizer())
        RCLCPP_INFO(logger_, "\033[41;37m Minco optimizer time:%f \033[0m", (ros::Time::now()-current).toSec());
    else
        return false;

    minco_.getTrajectory(optimizer_traj_);
    final_collision = check_final_collision(optimizer_traj_, iniStateXYTheta_);
    if(final_collision){
        penaltyWt_.time_weight *= 0.75;
        // safeDis *= 1.2;
    }
    else{
        break;
    }
  }
  Collision_point_Pub();
  penaltyWt_.time_weight = penaltyWt_.time_weight_backup_for_replan;
  safeDis = safeDis_;
  if(replan_num_for_coll == safeReplanMaxTime){
    ROS_ERROR("\n\n\n final traj Collision!!!!!!!!!!!\n\n\n\n");
    return false;
  }

  final_traj_ = optimizer_traj_;
  final_initStateXYTheta_ = iniStateXYTheta_;
  final_finStateXYTheta_ = finStateXYTheta_;
  Collision_point_Pub();

  return true;
}

bool MincoSmoother::get_state(const FlatTrajData &flat_traj){
  TrajNum_ = flat_traj.UnOccupied_traj_pts.size()+1;

  Innerpoints_.resize(2, TrajNum_-1);
  for(u_int i=0; i<flat_traj.UnOccupied_traj_pts.size(); i++){
      Innerpoints_.col(i) = flat_traj.UnOccupied_traj_pts[i].head(2);
  }

  inner_init_positions__ = flat_traj.UnOccupied_positions;
  inner_init_positions__.push_back(flat_traj.final_state_XYTheta);

  iniState_ = flat_traj.start_state;
  finState_ = flat_traj.final_state;

  pieceTime_.resize(TrajNum_); pieceTime_.setOnes();
  pieceTime_ *= flat_traj.UnOccupied_initT;

  iniStateXYTheta_ = flat_traj.start_state_XYTheta;
  finStateXYTheta_ = flat_traj.final_state_XYTheta;
  
  return true;
}

bool MincoSmoother::optimizer(){
  EqualLambda = init_EqualLambda_;
  EqualRho = init_EqualRho_; 

  // 2*(N-1) intermediate points, 1 relaxed S, N times
  int variable_num_ = 3 * TrajNum_ - 1;

  // RCLCPP_INFO_STREAM(logger_, "iniStates: \n" << iniState_);
  // RCLCPP_INFO_STREAM(logger_, "finStates: \n" << finState_);
  // RCLCPP_INFO(logger_, "TrajNum: %d", TrajNum_);

  minco_.setConditions(iniState_, finState_, TrajNum_, energyWeights_);
  // RCLCPP_INFO_STREAM(logger_, "init Innerpoints: \n" << Innerpoints_);
  // RCLCPP_INFO_STREAM(logger_, "init pieceTime_: " << pieceTime_.transpose());
  minco_.setParameters(Innerpoints_, pieceTime_);   
  minco_.getTrajectory(init_final_traj_);
  Eigen::VectorXd x;
  x.resize(variable_num_);
  int offset = 0;
  memcpy(x.data()+offset,Innerpoints_.data(), Innerpoints_.size() * sizeof(x[0]));
  offset += Innerpoints_.size();
  x[offset] = finState_(1,0);
  ++offset;
  Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, pieceTime_.size());
  offset += pieceTime_.size();
  RealT2VirtualT(pieceTime_,Vt);

  double cost;
  int result;
  Eigen::VectorXd g;
  g.resize(x.size());
  iter_num_ = 0;

  auto start = std::chrono::high_resolution_clock::now();
  // Handle cases where the path is too short to converge
  if (fabs(finState_(1, 0)) < path_lbfgs_params_.shot_path_horizon) {
      path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.shot_path_past;
  } else {
      path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.normal_past;
  }

  ifprint = false;
  result = lbfgs::lbfgs_optimize(x,
                                 cost,
                                 MincoSmoother::costFunctionCallbackPath,
                                 NULL,
                                 NULL,
                                 this,
                                 path_lbfgs_params_.path_lbfgs_params);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  // Output computation time
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "pre_process";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 11.5;
  marker.pose.position.y = 7;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  double search_time = duration / 1000.0;
  std::ostringstream out;
  out << std::fixed << "Pre-process: \n"<< std::setprecision(2) << search_time<<" ms";
  marker.text = out.str();
  recordTextPub.publish(marker);

  ifprint = true;
  MincoSmoother::costFunctionCallbackPath(this,x,g);
  ifprint = false;

  RCLCPP_INFO_STREAM(logger_, "Pre-processing optimizer:" << duration / 1000.0 << " ms");
  RCLCPP_INFO(logger_, "Pre-processing finish! result:%d, finalcost:%f, iter_num_:%d", result, cost, iter_num_);
  offset = 0;
  Eigen::Map<Eigen::MatrixXd> PathP(x.data() + offset, 2, TrajNum_ - 1);
  offset += 2 * (TrajNum_ - 1);
  finalInnerpoints_ = PathP;
  finState_(1, 0) = x[offset];
  ++offset;

  Eigen::Map<const Eigen::VectorXd> Patht(x.data() + offset, TrajNum_);
  offset += TrajNum_;
  VirtualT2RealT(Patht, finalpieceTime_);
  minco_.setTConditions(finState_);
  minco_.setParameters(finalInnerpoints_, finalpieceTime_);
  minco_.getTrajectory(final_traj_);
  mincoPathPub(final_traj_, iniStateXYTheta_, pathmincoinitPath);
  mincoPointPub(final_traj_, iniStateXYTheta_, pathmincoinitPoint, Eigen::Vector3d(114, 159, 207));

  // RCLCPP_INFO_STREAM("Path final pieces time: " << finalpieceTime_.transpose());
  // RCLCPP_INFO_STREAM("Path final Innerpoints: \n" << finalInnerpoints_);
  // RCLCPP_INFO_STREAM("Path final finState: \n" << finState_);

  // Formal optimization
  // RCLCPP_INFO("---------------------------------------------------------------------init---------------------------------------------------------------");
  // ifprint = true;
  // costFunctionCallback(this, x, g);
  // ifprint = false;
  RCLCPP_INFO(logger_, "-------------------------------------------------------------------optimize---------------------------------------------------------------");
  iter_num_ = 0;
  
  rclcpp::Time current = rclcpp::Clock().now();

  while(rclcpp::ok()){
    result = lbfgs::lbfgs_optimize(x,
                                   cost,
                                   MincoSmoother::costFunctionCallback,
                                   NULL,
                                   MincoSmoother::earlyExit,
                                   this,
                                   lbfgs_params_);
    if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
      result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION){
      RCLCPP_INFO(logger_, "optimizer finish! result:%d   finalcost:%f   iter_num_:%d ",result,cost,iter_num_);
    } 
    else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
      RCLCPP_WARN(logger_, "Lbfgs: The line-search routine reaches the maximum number of evaluations.");
    }
    else{
      RCLCPP_WARN(logger_, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
    }

    // ALM update
    // RCLCPP_INFO_STREAM("EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
    if(FinalIntegralXYError.norm() < EqualTolerance_[0]){
        break;
    }
    EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
    EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
    EqualRho[0] = std::min((1 + EqualGamma_[0]) * EqualRho[0], EqualRhoMax_[0]);
    EqualRho[1] = std::min((1 + EqualGamma_[1]) * EqualRho[1], EqualRhoMax_[1]);
  }

  double mincotime = (rclcpp::Clock().now()-current).seconds();
  RCLCPP_INFO(logger_, "\033[40;36m minco_ optimizer time:%f \033[0m", mincotime);
  RCLCPP_INFO_STREAM(logger_, "--------------------------------------------------------------------final------------------------------------------------------");

  marker.header.frame_id = "world";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "minco_";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 11.5;
  marker.pose.position.y = 6;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  search_time = mincotime*1000.0;
  std::ostringstream out2;
  out2 << std::fixed <<"Optimization: \n"<< std::setprecision(2) << search_time << " ms";
  marker.text = out2.str();
  recordTextPub.publish(marker);

  // Output optimization infomation
  // ifprint = true;
  // costFunctionCallback(this,x,g);
  
  offset = 0;
  Eigen::Map<Eigen::MatrixXd> P(x.data()+offset, 2, TrajNum_ - 1);
  offset += 2 * (TrajNum_ - 1);
  finalInnerpoints_ = P;
  
  finState_(1,0) = x[offset];
  ++offset;

  Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, TrajNum_);
  offset += TrajNum_;
  VirtualT2RealT(t,finalpieceTime_);
  minco_.setTConditions(finState_);
  minco_.setParameters(finalInnerpoints_, finalpieceTime_);

  // std::cout<<"finalInnerpoints: \n"<<finalInnerpoints<<std::endl;
  // std::cout<<"finalpieceTime: \n"<<finalpieceTime.transpose()<<std::endl;

  RCLCPP_INFO(logger_, "\n\n optimizer finish! result:%d   finalcost:%f   iter_num_:%d\n\n ",result,cost,iter_num_);

  return true;
}

inline int MincoSmoother::earlyExit(void *instance,
                                    const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &g,
                                    const double fx,
                                    const double step,
                                    const int k,
                                    const int ls){
  MincoSmoother &obj = *(MincoSmoother *)instance;
  obj.FinalIntegralXYError_ = obj.FinalIntegralXYError;
  obj.collision_point_ = obj.collision_point;
  // std::cout<<"cost: "<<fx<<std::endl;

  if(obj.if_visual_optimization_){
    RCLCPP_INFO(logger_, "fx: %f  step: %f  k: %d  ls: %d", fx, step, k, ls);
    RCLCPP_INFO_STREAM(logger_, "x: " << x.transpose());
    RCLCPP_INFO_STREAM(logger_, "g: " << g.transpose());
    RCLCPP_INFO_STREAM(logger_, "");
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum_ - 1);
    offset += 2 * (obj.TrajNum_ - 1);
    obj.finalInnerpoints_ = P;

    obj.finState_(1,0) = x[offset];
    ++offset;

    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum_);
    offset += obj.TrajNum_;
    VirtualT2RealT(t,obj.finalpieceTime_);
    obj.minco_.setTConditions(obj.finState_);
    obj.minco_.setParameters(obj.finalInnerpoints_, obj.finalpieceTime_);
    obj.minco_.getTrajectory(obj.optimizer_traj_);
    // obj.mincoPathPub(obj.optimizer_traj_, obj.iniStateXYTheta_, obj.processmincoinitPath);
    rclcpp::Duration(0.5).sleep();
  }
  return 0;
}

double MincoSmoother::costFunctionCallback(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g){
  if(x.norm()>1e4)
      return inf;

  MincoSmoother &obj = *(MincoSmoother *)ptr;
  obj.iter_num_ += 1;

  g.setZero();
  // Map the input variables to the variable matrix
  int offset = 0;
  Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum_ - 1);
  Eigen::Map<Eigen::MatrixXd> gradP(g.data()+offset, 2, obj.TrajNum_ - 1);
  offset += 2 * (obj.TrajNum_ - 1);

  double* gradTailS = g.data()+offset;
  obj.finState_(1,0) = x[offset];
  ++offset;
  
  gradP.setZero();
  obj.Innerpoints_ = P;
  
  Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum_);
  Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.TrajNum_);

  offset += obj.TrajNum_;
  VirtualT2RealT(t, obj.pieceTime_);
  gradt.setZero();

  double cost;
  obj.minco_.setTConditions(obj.finState_);
  obj.minco_.setParameters(obj.Innerpoints_,obj.pieceTime_);
  obj.minco_.getEnergy(cost);
  obj.minco_.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs_);
  obj.minco_.getEnergyPartialGradByTimes(obj.partialGradByTimes_);
  if(obj.ifprint){
    RCLCPP_INFO(logger_, "Energy cost: %f", cost);
  }
  obj.attachPenaltyFunctional(cost);
  if(obj.ifprint){
    RCLCPP_INFO(logger_, "attachPenaltyFunctional cost: %f", cost);
  }
  obj.minco_.propogateArcYawLenghGrad(obj.partialGradByCoeffs_, obj.partialGradByTimes_,
                                      obj.gradByPoints_, obj.gradByTimes_, obj.gradByTailStateS_);
  cost += obj.penaltyWt_.time_weight * obj.pieceTime_.sum();

  if(obj.ifprint){
    RCLCPP_INFO(logger_, "T cost: %f", obj.penaltyWt_.time_weight * obj.pieceTime_.sum());
  }
  Eigen::VectorXd rhotimes;
  rhotimes.resize(obj.gradByTimes_.size());
  obj.gradByTimes_ += obj.penaltyWt_.time_weight * rhotimes.setOnes();

  *gradTailS = obj.gradByTailStateS_.y();

  gradP = obj.gradByPoints_;
  backwardGradT(t, obj.gradByTimes_, gradt);
  
  return cost;
}

void MincoSmoother::attachPenaltyFunctional(double &cost){
  collision_point.clear();
  double ini_x = iniStateXYTheta_.x();
  double ini_y = iniStateXYTheta_.y();

  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
  double s1, s2, s3, s4, s5;
  Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
  double IntegralAlpha, Alpha, omg, omgstep;
  
  double unoccupied_averageT;
  unoccupied_averageT = pieceTime_.mean();
  
  double cost_corrb=0, cost_v=0, cost_a=0, cost_omega=0, cost_domega=0, cost_endp=0, cost_moment=0, cost_meanT=0, cost_centripetal_acc=0;
  
  double violaAcc, violaAlp, violaPos, violaMom, violaCenAcc;
  double violaAccPena, violaAlpPena, violaPosPena, violaMomPena, violaCenAccPena;
  double violaAccPenaD, violaAlpPenaD, violaPosPenaD, violaMomPenaD, violaCenAccPenaD;
  double gradViolaAT, gradViolaDOT, gradViolaPt, gradViolaMt, gradViolaCAt;

  double violaVel, violaVelPena, violaVelPenaD;
  double violaOmega, violaOmegaPena, violaOmegaPenaD;

  Eigen::Matrix2d help_L;
  Eigen::Vector3d gradESDF;
  Eigen::Vector2d gradESDF2d;

  // Used to obtain the position of each integral point
  std::vector<Eigen::VectorXd> VecIntegralX;
  std::vector<Eigen::VectorXd> VecIntegralY;
  std::vector<Eigen::Vector2d> VecTrajFinalXY;
  VecTrajFinalXY.emplace_back(ini_x, ini_y);

  // Store derivatives for chain rule
  std::vector<Eigen::MatrixXd> VecSingleXGradCS;
  std::vector<Eigen::MatrixXd> VecSingleXGradCTheta;
  std::vector<Eigen::VectorXd> VecSingleXGradT;
  std::vector<Eigen::MatrixXd> VecSingleYGradCS;
  std::vector<Eigen::MatrixXd> VecSingleYGradCTheta;
  std::vector<Eigen::VectorXd> VecSingleYGradT;

  Eigen::MatrixXd SingleXGradCS(6,SamNumEachPart_+1);
  Eigen::MatrixXd SingleXGradCTheta(6,SamNumEachPart_+1);
  Eigen::VectorXd SingleXGradT(SamNumEachPart_+1);
  Eigen::MatrixXd SingleYGradCS(6,SamNumEachPart_+1);
  Eigen::MatrixXd SingleYGradCTheta(6,SamNumEachPart_+1);
  Eigen::VectorXd SingleYGradT(SamNumEachPart_+1);
  Eigen::VectorXd IntegralX(sparseResolution_);
  Eigen::VectorXd IntegralY(sparseResolution_);

  // Used to store the positions obtained by integration
  Eigen::VectorXd VecCoeffChainX(TrajNum_*(SamNumEachPart_+1));VecCoeffChainX.setZero();
  Eigen::VectorXd VecCoeffChainY(TrajNum_*(SamNumEachPart_+1));VecCoeffChainY.setZero();
  Eigen::Vector2d CurrentPointXY(ini_x, ini_y);

  for(int i=0; i<TrajNum_; i++){
    const Eigen::Matrix<double, 6, 2> &c = minco_.getCoeffs().block<6,2>(6*i, 0);
    double step = pieceTime_[i] / sparseResolution_;
    double halfstep = step / 2.0;
    double CoeffIntegral = pieceTime_[i] / sparseResolution_6_;
    
    IntegralX.setZero();
    IntegralY.setZero();
    
    s1 = 0.0;

    for(int j=0; j<=SamNumEachPart_; j++){
      if(j%2 == 0){
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s3 * s2;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        s1 += halfstep;        
        IntegralAlpha = 1.0 / SamNumEachPart_ * j;
        Alpha = 1.0 / sparseResolution_ * (double(j)/2); 
        omg = (j==0||j==SamNumEachPart_)? 0.5:1;
        omgstep = omg * step;
        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;
        // Store gradients for beta0, beta1, and beta2 to simplify calculations. Columns represent yaw and s, rows represent beta0, beta1, and beta2.
        Eigen::MatrixXd gradBeta;gradBeta.resize(3,2);gradBeta.setZero();
        // Store cos and sin to simplify calculations
        double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());
        

        if(if_standard_diff_){
          if(j!=0){
            IntegralX[j/2-1] += CoeffIntegral * dsigma.y() * cosyaw;
            IntegralY[j/2-1] += CoeffIntegral * dsigma.y() * sinyaw;
          }
          if(j!=SamNumEachPart_){
            IntegralX[j/2] += CoeffIntegral * dsigma.y() * cosyaw;
            IntegralY[j/2] += CoeffIntegral * dsigma.y() * sinyaw;
          }

          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
        }
        else{
          if(j!=0){
            IntegralX[j/2-1] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
            IntegralY[j/2-1] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
          }
          if(j!=SamNumEachPart_){
            IntegralX[j/2] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
            IntegralY[j/2] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
          }

          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                          + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                          + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                          - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                          + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
        }


        violaAcc = ddsigma.y()*ddsigma.y() - config_.max_acc_ * config_.max_acc_;
        violaAlp = ddsigma.x()*ddsigma.x() - config_.max_domega_ * config_.max_domega_;

        if(violaAcc > 0){
          positiveSmoothedL1(smoothEps_, violaAcc, violaAccPena, violaAccPenaD);
          gradViolaAT = 2.0 * Alpha * ddsigma.y() * dddsigma.y();
          gradBeta(2,1) +=  omgstep * penaltyWt_.acc_weight * violaAccPenaD * 2.0 * ddsigma.y();
          partialGradByTimes_(i) += omg * penaltyWt_.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / sparseResolution_);
          cost += omgstep * penaltyWt_.acc_weight * violaAccPena;
          cost_a += omgstep * penaltyWt_.acc_weight * violaAccPena;
        }
        if(violaAlp > 0){
          positiveSmoothedL1(smoothEps_, violaAlp, violaAlpPena, violaAlpPenaD);
          gradViolaDOT = 2.0 * Alpha * ddsigma.x() * dddsigma.x();
          gradBeta(2,0) += omgstep * penaltyWt_.domega_weight * violaAlpPenaD * 2.0 * ddsigma.x();
          partialGradByTimes_(i) += omg * penaltyWt_.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / sparseResolution_);
          cost += omgstep * penaltyWt_.domega_weight * violaAlpPena;
          cost_domega += omgstep * penaltyWt_.domega_weight * violaAlpPena;
        }

        if(config_.if_directly_constrain_v_omega_){
          // Directly constrain velocity and angular velocity
          violaVel = dsigma.y() * dsigma.y() - config_.max_vel_ * config_.max_vel_;
          if(violaVel > 0){
            positiveSmoothedL1(smoothEps_, violaVel, violaVelPena, violaVelPenaD);
            gradViolaPt = 2.0 * Alpha * dsigma.y()
                            * ddsigma.y();
            gradBeta(1,1) += omgstep * penaltyWt_.moment_weight * violaVelPenaD * 2.0 * dsigma.y();
            partialGradByTimes_(i) += omg * penaltyWt_.moment_weight * (violaVelPenaD * gradViolaPt * step + violaVelPena / sparseResolution_);
            cost += omgstep * penaltyWt_.moment_weight * violaVelPena;
            cost_moment += omgstep * penaltyWt_.moment_weight * violaVelPena;
          }
          violaOmega = dsigma.x() * dsigma.x() - config_.max_omega_ * config_.max_omega_;
          if(violaOmega > 0){
            positiveSmoothedL1(smoothEps_, violaOmega, violaOmegaPena, violaOmegaPenaD);
            gradViolaPt = 2.0 * Alpha * dsigma.x()
                            * ddsigma.x();
            gradBeta(1,0) += omgstep * penaltyWt_.moment_weight * violaOmegaPenaD * 2.0 * dsigma.x();
            partialGradByTimes_(i) += omg * penaltyWt_.moment_weight * (violaOmegaPenaD * gradViolaPt * step + violaOmegaPena / sparseResolution_);
            cost += omgstep * penaltyWt_.moment_weight * violaOmegaPena;
            cost_moment += omgstep * penaltyWt_.moment_weight * violaOmegaPena;
          }
        }
        else{
          // Handle the constraints on speed and angular velocity caused by the driving wheel torque. 
          // The polynomial inequality forms a symmetric quadrilateral, so four hyperplanes are used for constraint.
          for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
            violaMom = omg_sym * config_.max_vel_ * dsigma.x() + config_.max_omega_ * dsigma.y() - config_.max_vel_ * config_.max_omega_;
            if(violaMom > 0){
              positiveSmoothedL1(smoothEps_, violaMom, violaMomPena, violaMomPenaD);
              gradViolaMt = Alpha * (omg_sym * config_.max_vel_ * ddsigma.x() + config_.max_omega_ * ddsigma.y());
              gradBeta(1,0) += omgstep * penaltyWt_.moment_weight * violaMomPenaD * omg_sym * config_.max_vel_;
              gradBeta(1,1) += omgstep * penaltyWt_.moment_weight * violaMomPenaD * config_.max_omega_;
              partialGradByTimes_(i) += omg * penaltyWt_.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
              cost += omgstep * penaltyWt_.moment_weight * violaMomPena;
              cost_moment += omgstep * penaltyWt_.moment_weight * violaMomPena;
            }
          }
          for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
            violaMom = omg_sym * -config_.min_vel_ * dsigma.x() - config_.max_omega_ * dsigma.y() + config_.min_vel_ * config_.max_omega_;
            if(violaMom > 0){
              positiveSmoothedL1(smoothEps_, violaMom, violaMomPena, violaMomPenaD);
              gradViolaMt = Alpha * (omg_sym * -config_.min_vel_ * ddsigma.x() - config_.max_omega_ * ddsigma.y());
              gradBeta(1,0) += omgstep * penaltyWt_.moment_weight * violaMomPenaD * omg_sym * -config_.min_vel_;
              gradBeta(1,1) -= omgstep * penaltyWt_.moment_weight * violaMomPenaD * config_.max_omega_;
              partialGradByTimes_(i) += omg * penaltyWt_.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
              cost += omgstep * penaltyWt_.moment_weight * violaMomPena;
              cost_moment += omgstep * penaltyWt_.moment_weight * violaMomPena;
            }
          }
        }
        // Anti-skid or anti-rollover constraint
        violaCenAcc = dsigma.x()*dsigma.x()*dsigma.y()*dsigma.y() - config_.max_centripetal_acc_*config_.max_centripetal_acc_;
        if(violaCenAcc > 0){
          positiveSmoothedL1(smoothEps_, violaCenAcc, violaCenAccPena, violaCenAccPenaD);
          gradViolaCAt = 2.0 * Alpha * (dsigma.x() * dsigma.y() * dsigma.y() * ddsigma.x() + dsigma.y() * dsigma.x() * dsigma.x() * ddsigma.y());
          gradBeta(1,0) += omgstep * penaltyWt_.cen_acc_weight * violaCenAccPenaD * (2 * dsigma.x() * dsigma.y() * dsigma.y());
          gradBeta(1,1) += omgstep * penaltyWt_.cen_acc_weight * violaCenAccPenaD * (2 * dsigma.x() * dsigma.x() * dsigma.y());
          partialGradByTimes_(i) += omg * penaltyWt_.cen_acc_weight * (violaCenAccPenaD * gradViolaCAt * step + violaCenAccPena / sparseResolution_);
          cost += omgstep * penaltyWt_.cen_acc_weight * violaCenAccPena;
          cost_centripetal_acc += omgstep * penaltyWt_.cen_acc_weight * violaCenAccPena;
        }

        // Collision constraint
        if(j != 0) CurrentPointXY+=Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);

        Eigen::Matrix2d ego_R;
        ego_R << cosyaw,-sinyaw, sinyaw, cosyaw;

        bool if_collision = false;
        Eigen::Vector2d all_grad2Pos; all_grad2Pos.setZero();


        for(auto cp2D:check_point){
          Eigen::Vector2d bpt = CurrentPointXY + ego_R * cp2D;
          double sdf_value = map_->getDistWithGradBilinear(bpt, gradESDF2d, safeDis);
          violaPos = -sdf_value + safeDis;
          if (violaPos > 0.0){
            if_collision = true;
            positiveSmoothedL1(smoothEps_, violaPos, violaPosPena, violaPosPenaD);
            all_grad2Pos -= omgstep * penaltyWt_.collision_weight * violaPosPenaD * gradESDF2d;
            help_L << -sinyaw, -cosyaw, cosyaw, -sinyaw;
            gradViolaPt = -Alpha * dsigma.x() * gradESDF2d.transpose() * help_L * cp2D;

            gradBeta(0, 0) -= omgstep * penaltyWt_.collision_weight * violaPosPenaD * gradESDF2d.transpose() * help_L * cp2D;
            partialGradByTimes_(i) += omg * penaltyWt_.collision_weight * (violaPosPenaD * gradViolaPt * step + violaPosPena / sparseResolution_);
            cost += omgstep * penaltyWt_.collision_weight * violaPosPena;
            cost_corrb += omgstep * penaltyWt_.collision_weight * violaPosPena;
          }
        }
        if(if_collision){
          VecCoeffChainX.head(i*(SamNumEachPart_+1)+j+1).array() += all_grad2Pos.x();
          VecCoeffChainY.head(i*(SamNumEachPart_+1)+j+1).array() += all_grad2Pos.y();
        }

        partialGradByCoeffs_.block<6,2>(i*6, 0) += beta0 * gradBeta.row(0) + beta1 * gradBeta.row(1) + beta2 * gradBeta.row(2);
      }

      else{
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s3 * s2;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        s1 += halfstep;
        IntegralAlpha = 1.0 / SamNumEachPart_ * j;
        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());

        if(if_standard_diff_){
          IntegralX[j/2] += 4 * CoeffIntegral * dsigma.y() * cosyaw;
          IntegralY[j/2] += 4 * CoeffIntegral * dsigma.y() * sinyaw;
          
          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;
          
          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
        }
        else{
          IntegralX[j/2] += 4 * CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
          IntegralY[j/2] += 4 * CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
          
          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                              + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                          + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                              - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                          + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
        }            
      }
    }

    // segment duration balance
    if( pieceTime_[i] < unoccupied_averageT * mean_time_lowBound_){
      cost += penaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_);
      cost_meanT += penaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_);
      partialGradByTimes_.array() += penaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_)  * (- mean_time_lowBound_ / TrajNum_);
      partialGradByTimes_(i) += penaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_);
    }
    if (pieceTime_[i] > unoccupied_averageT * mean_time_uppBound_){
      cost += penaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_);
      cost_meanT += penaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_);
      partialGradByTimes_.array() += penaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_) * (-mean_time_uppBound_ / TrajNum_);
      partialGradByTimes_(i) += penaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_);
    }

    VecIntegralX.push_back(IntegralX);
    VecIntegralY.push_back(IntegralY);
    VecTrajFinalXY.push_back(VecTrajFinalXY[i] + Eigen::Vector2d(IntegralX.sum(), IntegralY.sum()));
    ///////////////////////////////////////////////////////////////////////////
    VecSingleXGradCS.push_back(SingleXGradCS * CoeffIntegral);
    VecSingleXGradCTheta.push_back(SingleXGradCTheta * CoeffIntegral);
    VecSingleXGradT.push_back(SingleXGradT);
    VecSingleYGradCS.push_back(SingleYGradCS * CoeffIntegral);
    VecSingleYGradCTheta.push_back(SingleYGradCTheta * CoeffIntegral);
    VecSingleYGradT.push_back(SingleYGradT);
    ///////////////////////////////////////////////////////////////////////////
  }

  // final position constraint
  FinalIntegralXYError = VecTrajFinalXY.back() - finStateXYTheta_.head(2);
  cost += 0.5 * (EqualRho[0] * pow(FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0], 2) + EqualRho[1] * pow(FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1], 2));
  cost_endp += 0.5 * (EqualRho[0] * pow(FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0], 2) + EqualRho[1] * pow(FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1], 2));
  if(ifprint){
    RCLCPP_INFO(logger_, "\033[40;33m iter finStateXY:%f  %f  \033[0m", VecTrajFinalXY.back().x(), VecTrajFinalXY.back().y());
    RCLCPP_INFO(logger_, "\033[40;33m real finStateXY:%f  %f  \033[0m", finStateXYTheta_.x(), finStateXYTheta_.y());
    RCLCPP_INFO(logger_, "error: %f", FinalIntegralXYError.norm());
  }
  VecCoeffChainX.array() += EqualRho[0] * (FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0]);
  VecCoeffChainY.array() += EqualRho[1] * (FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1]);


  if(ifprint){
    RCLCPP_INFO(logger_, "cost: %f", cost);
    RCLCPP_INFO(logger_, "cost corridor: %f", cost_corrb);
    RCLCPP_INFO(logger_, "cost end p: %f", cost_endp);
    RCLCPP_INFO(logger_, "cost v: %f", cost_v);
    RCLCPP_INFO(logger_, "cost a: %f", cost_a);
    RCLCPP_INFO(logger_, "cost omega: %f", cost_omega);
    RCLCPP_INFO(logger_, "cost domega: %f", cost_domega);
    RCLCPP_INFO(logger_, "cost moment: %f", cost_moment);
    RCLCPP_INFO(logger_, "cost meanT: %f", cost_meanT);
    RCLCPP_INFO(logger_, "cost centripetal_acc: %f", cost_centripetal_acc);
  } 

  // Push the coefficients to the gradient, note that this part must be after the final state constraints and collision constraints
  for(int i=0; i<TrajNum_; i++){
    ///////////////////////////////////////////////////////////////////////////
    Eigen::VectorXd CoeffX = VecCoeffChainX.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_);
    Eigen::VectorXd CoeffY = VecCoeffChainY.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_);
    
    partialGradByCoeffs_.block<6,1>(i*6, 1) += VecSingleXGradCS[i] * CoeffX;
    partialGradByCoeffs_.block<6,1>(i*6, 0) += VecSingleXGradCTheta[i] * CoeffX;
    partialGradByCoeffs_.block<6,1>(i*6, 1) += VecSingleYGradCS[i] * CoeffY;
    partialGradByCoeffs_.block<6,1>(i*6, 0) += VecSingleYGradCTheta[i] * CoeffY;
    ///////////////////////////////////////////////////////////////////////////
    partialGradByTimes_(i) += (VecSingleXGradT[i].cwiseProduct(CoeffX)).sum();
    partialGradByTimes_(i) += (VecSingleYGradT[i].cwiseProduct(CoeffY)).sum();
  }
}

double MincoSmoother::costFunctionCallbackPath(void *ptr,
                                         const Eigen::VectorXd &x,
                                         Eigen::VectorXd &g){
  if(x.norm()>1e4){
      return inf;
  }
  MincoSmoother &obj = *(MincoSmoother *)ptr;
  ++obj.iter_num_;
  int offset = 0;
  Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum_ - 1);
  Eigen::Map<Eigen::MatrixXd> gradP(g.data()+offset, 2, obj.TrajNum_ - 1);
  offset += 2 * (obj.TrajNum_ - 1);

  double* gradTailS = g.data()+offset;
  obj.finState_(1,0) = x[offset];
  ++offset;

  gradP.setZero();
  obj.Innerpoints_ = P;
  Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum_);
  Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.TrajNum_);
  offset += obj.TrajNum_;
  VirtualT2RealT(t, obj.pieceTime_);
  gradt.setZero();
  double cost;
  obj.minco_.setTConditions(obj.finState_);
  obj.minco_.setParameters(obj.Innerpoints_,obj.pieceTime_);
  obj.minco_.getEnergy(cost);
  obj.minco_.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs_);
  obj.minco_.getEnergyPartialGradByTimes(obj.partialGradByTimes_);
  obj.attachPenaltyFunctionalPath(cost);
  obj.minco_.propogateArcYawLenghGrad(obj.partialGradByCoeffs_, obj.partialGradByTimes_,
                                      obj.gradByPoints, obj.gradByTimes_, obj.gradByTailStateS);

  *gradTailS = obj.gradByTailStateS.y();

  cost += obj.PathpenaltyWt_.time_weight * obj.pieceTime_.sum();

  Eigen::VectorXd rhotimes;
  rhotimes.resize(obj.gradByTimes_.size());
  obj.gradByTimes_ += obj.penaltyWt_.time_weight * rhotimes.setOnes();
  gradP = obj.gradByPoints;
  backwardGradT(t, obj.gradByTimes_, gradt);
  
  return cost;
}

void MincoSmoother::attachPenaltyFunctionalPath(double &cost){
  double ini_x = iniStateXYTheta_.x();
  double ini_y = iniStateXYTheta_.y();

  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
  double s1, s2, s3, s4, s5;
  Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
  int SamNumEachPart_ = 2 * sparseResolution_;
  double IntegralAlpha, omg;

  double unoccupied_averageT;
  unoccupied_averageT = pieceTime_.mean();
  
  double violaPos;

  double violaMom;
  double violaMomPena;
  double violaMomPenaD;

  double cost_bp=0, cost_final_p=0, cost_moment=0, cost_meanT=0;

  Eigen::Matrix2d help_L;
  Eigen::Vector2d gradESDF2d;

  Eigen::VectorXd IntegralChainCoeff_(SamNumEachPart_ + 1);
  IntegralChainCoeff_.setZero();
  for(int i=0; i<sparseResolution_; i++){
      IntegralChainCoeff_.block(2*i,0,3,1) += Eigen::Vector3d(1.0, 4.0, 1.0);
  }

  std::vector<Eigen::VectorXd> VecIntegralX(TrajNum_);
  std::vector<Eigen::VectorXd> VecIntegralY(TrajNum_);
  std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum_+1);
  VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

  std::vector<Eigen::MatrixXd> VecSingleXGradCS(TrajNum_);
  std::vector<Eigen::MatrixXd> VecSingleXGradCTheta(TrajNum_);
  std::vector<Eigen::VectorXd> VecSingleXGradT(TrajNum_);
  std::vector<Eigen::MatrixXd> VecSingleYGradCS(TrajNum_);
  std::vector<Eigen::MatrixXd> VecSingleYGradCTheta(TrajNum_);
  std::vector<Eigen::VectorXd> VecSingleYGradT(TrajNum_);

  Eigen::VectorXd VecCoeffChainX(TrajNum_*(SamNumEachPart_+1));VecCoeffChainX.setZero();
  Eigen::VectorXd VecCoeffChainY(TrajNum_*(SamNumEachPart_+1));VecCoeffChainY.setZero();
  // Eigen::Vector2d CurrentPointXY(ini_x, ini_y);

  for(int i=0; i<TrajNum_; i++){
    const Eigen::Matrix<double, 6, 2> &c = minco_.getCoeffs().block<6,2>(6*i, 0);
    double step = pieceTime_[i] / sparseResolution_;
    double halfstep = step / 2;
    double CoeffIntegral = pieceTime_[i] / sparseResolution_ / 6;
    Eigen::MatrixXd SingleXGradCS(6,SamNumEachPart_+1);
    Eigen::MatrixXd SingleXGradCTheta(6,SamNumEachPart_+1);
    Eigen::VectorXd SingleXGradT(SamNumEachPart_+1);
    Eigen::MatrixXd SingleYGradCS(6,SamNumEachPart_+1);
    Eigen::MatrixXd SingleYGradCTheta(6,SamNumEachPart_+1);
    Eigen::VectorXd SingleYGradT(SamNumEachPart_+1);

    Eigen::VectorXd IntegralX(sparseResolution_);IntegralX.setZero();
    Eigen::VectorXd IntegralY(sparseResolution_);IntegralY.setZero();
    s1 = 0.0;
    for(int j=0; j<=SamNumEachPart_; j++){
      if(j%2 == 0){
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s3 * s2;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        s1 += halfstep;        
        IntegralAlpha = 1.0 / SamNumEachPart_ * j;
        omg = (j==0||j==SamNumEachPart_)? 0.5:1;
        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;
        double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());

        if(if_standard_diff_){
          if(j!=0){
              IntegralX[j/2-1] += CoeffIntegral * dsigma.y() * cosyaw;
              IntegralY[j/2-1] += CoeffIntegral * dsigma.y() * sinyaw;
          }
          if(j!=SamNumEachPart_){
              IntegralX[j/2] += CoeffIntegral * dsigma.y() * cosyaw;
              IntegralY[j/2] += CoeffIntegral * dsigma.y() * sinyaw;
          }

          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
        }
        else{
          if(j!=0){
              IntegralX[j/2-1] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
              IntegralY[j/2-1] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
          }
          if(j!=SamNumEachPart_){
              IntegralX[j/2] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
              IntegralY[j/2] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
          }

          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                              + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                          + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                              - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                          + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
        }

        // Path similarity constraint
        // if(j != 0) CurrentPointXY+=Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);
        
        double gradViolaMt;
        double Alpha = 1.0 / sparseResolution_ * (double(j)/2); 
        Eigen::MatrixXd gradBeta;gradBeta.resize(3,2);gradBeta.setZero();
        for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
          violaMom = omg_sym * config_.max_vel_ * dsigma.x() + config_.max_omega_ * dsigma.y() - config_.max_vel_ * config_.max_omega_;
          if(violaMom > 0){
            positiveSmoothedL1(smoothEps_, violaMom, violaMomPena, violaMomPenaD);
            gradViolaMt = Alpha * (omg_sym * config_.max_vel_ * ddsigma.x() + config_.max_omega_ * ddsigma.y());
            gradBeta(1,0) += omg * step * PathpenaltyWt_.moment_weight * violaMomPenaD * omg_sym * config_.max_vel_;
            gradBeta(1,1) += omg * step * PathpenaltyWt_.moment_weight * violaMomPenaD * config_.max_omega_;
            partialGradByTimes_(i) += omg * PathpenaltyWt_.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
            cost += omg * step * PathpenaltyWt_.moment_weight * violaMomPena;
            cost_moment += omg * step * PathpenaltyWt_.moment_weight * violaMomPena;
          }
        }
        for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
          violaMom = omg_sym * -config_.min_vel_ * dsigma.x() - config_.max_omega_ * dsigma.y() + config_.min_vel_ * config_.max_omega_;
          if(violaMom > 0){
            positiveSmoothedL1(smoothEps_, violaMom, violaMomPena, violaMomPenaD);
            gradViolaMt = Alpha * (omg_sym * -config_.min_vel_ * ddsigma.x() - config_.max_omega_ * ddsigma.y());
            gradBeta(1,0) += omg * step * PathpenaltyWt_.moment_weight * violaMomPenaD * omg_sym * -config_.min_vel_;
            gradBeta(1,1) -= omg * step * PathpenaltyWt_.moment_weight * violaMomPenaD * config_.max_omega_;
            partialGradByTimes_(i) += omg * PathpenaltyWt_.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
            cost += omg * step * PathpenaltyWt_.moment_weight * violaMomPena;
            cost_moment += omg * step * PathpenaltyWt_.moment_weight * violaMomPena;
          }
        }

        double violaAcc = ddsigma.y()*ddsigma.y() - config_.max_acc_*config_.max_acc_;
        double violaAlp = ddsigma.x()*ddsigma.x() - config_.max_domega_*config_.max_domega_;
        double violaAccPena, violaAccPenaD, violaAlpPena, violaAlpPenaD;
        if(violaAcc > 0){
          positiveSmoothedL1(smoothEps_, violaAcc, violaAccPena, violaAccPenaD);
          double gradViolaAT = 2.0 * Alpha * ddsigma.y() * dddsigma.y();
          gradBeta(2,1) +=  omg * step * PathpenaltyWt_.acc_weight * violaAccPenaD * 2.0 * ddsigma.y();
          partialGradByTimes_(i) += omg * PathpenaltyWt_.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / sparseResolution_);
          cost += omg * step * PathpenaltyWt_.acc_weight * violaAccPena;
          cost_moment += omg * step * PathpenaltyWt_.acc_weight * violaAccPena;
        }
        if(violaAlp > 0){
          positiveSmoothedL1(smoothEps_, violaAlp, violaAlpPena, violaAlpPenaD);
          double gradViolaDOT = 2.0 * Alpha * ddsigma.x() * dddsigma.x();
          gradBeta(2,0) += omg * step * PathpenaltyWt_.domega_weight * violaAlpPenaD * 2.0 * ddsigma.x();
          partialGradByTimes_(i) += omg * PathpenaltyWt_.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / sparseResolution_);
          cost += omg * step * PathpenaltyWt_.domega_weight * violaAlpPena;
          cost_moment += omg * step * PathpenaltyWt_.domega_weight * violaAlpPena;
        }

        partialGradByCoeffs_.block<6,2>(i*6, 0) += beta0 * gradBeta.row(0) + beta1 * gradBeta.row(1) + beta2 * gradBeta.row(2);
      }
      else{
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s3 * s2;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        s1 += halfstep;
        IntegralAlpha = 1.0 / SamNumEachPart_ * j;
        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;

        double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());
        

        if(if_standard_diff_){
          IntegralX[j/2] += 4 * CoeffIntegral * dsigma.y() * cosyaw;
          IntegralY[j/2] += 4 * CoeffIntegral * dsigma.y() * sinyaw;
          
          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = -dsigma.y() * beta0 * sinyaw;
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * cosyaw /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = dsigma.y() * beta0 * cosyaw;
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw)*IntegralAlpha*CoeffIntegral + dsigma.y() * sinyaw /sparseResolution_6_;
        }
        else{
          IntegralX[j/2] += 4 * CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
          IntegralY[j/2] += 4 * CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
          
          SingleXGradCS.col(j) = beta1 * cosyaw;
          SingleXGradCTheta.col(j) = beta0 * (-dsigma.y() * sinyaw + dsigma.x() * ICR_.z() * cosyaw) + beta1 * sinyaw * ICR_.z(); 
          SingleXGradT[j] = (ddsigma.y() * cosyaw - dsigma.y() * dsigma.x() * sinyaw 
                              + ddsigma.x() * ICR_.z() * sinyaw + dsigma.x() * dsigma.x() * ICR_.z() * cosyaw)*IntegralAlpha*CoeffIntegral 
                          + (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw) /sparseResolution_6_;

          SingleYGradCS.col(j) = beta1 * sinyaw;
          SingleYGradCTheta.col(j) = beta0 * (dsigma.y() * cosyaw - dsigma.x() * ICR_.z() * sinyaw) - beta1 * cosyaw * ICR_.z();
          SingleYGradT[j] = (ddsigma.y() * sinyaw + dsigma.y() * dsigma.x() * cosyaw
                              - ddsigma.x() * ICR_.z() * cosyaw + dsigma.x() * dsigma.x() * ICR_.z() * sinyaw)*IntegralAlpha*CoeffIntegral
                          + (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw) /sparseResolution_6_;
        }
      }
    }
    VecIntegralX[i] = IntegralX;
    VecIntegralY[i] = IntegralY;
    VecTrajFinalXY[i+1] = VecTrajFinalXY[i] + Eigen::Vector2d(IntegralX.sum(), IntegralY.sum());
    VecSingleXGradCS[i] = SingleXGradCS * CoeffIntegral;
    VecSingleXGradCTheta[i] = SingleXGradCTheta * CoeffIntegral;
    VecSingleXGradT[i] = SingleXGradT ;
    VecSingleYGradCS[i] = SingleYGradCS * CoeffIntegral;
    VecSingleYGradCTheta[i] = SingleYGradCTheta * CoeffIntegral;
    VecSingleYGradT[i] = SingleYGradT;

    if( pieceTime_[i] < unoccupied_averageT * mean_time_lowBound_){
      cost += PathpenaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_);
      cost_meanT += PathpenaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_);
      partialGradByTimes_.array() += PathpenaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_)  * (- mean_time_lowBound_ / TrajNum_);
      partialGradByTimes_(i) += PathpenaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_lowBound_);
    }
    if( pieceTime_[i] > unoccupied_averageT * mean_time_uppBound_){
      cost += PathpenaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_);
      cost_meanT += PathpenaltyWt_.mean_time_weight * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_);
      partialGradByTimes_.array() += PathpenaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_)  * (- mean_time_uppBound_ / TrajNum_);
      partialGradByTimes_(i) += PathpenaltyWt_.mean_time_weight * 2.0 * (pieceTime_[i] - unoccupied_averageT * mean_time_uppBound_);
    }

    // Path point constraint
    Eigen::Vector2d innerpointXY = VecTrajFinalXY[i+1];
    violaPos = (innerpointXY - inner_init_positions_[i].head(2)).squaredNorm();
    VecCoeffChainX.head((i+1)*(SamNumEachPart_+1)).array() += PathpenaltyWt_.bigpath_sdf_weight * 2.0 * (innerpointXY.x() - inner_init_positions_[i].x());
    VecCoeffChainY.head((i+1)*(SamNumEachPart_+1)).array() += PathpenaltyWt_.bigpath_sdf_weight * 2.0 * (innerpointXY.y() - inner_init_positions_[i].y());
    cost += PathpenaltyWt_.bigpath_sdf_weight * violaPos;
    cost_bp += PathpenaltyWt_.bigpath_sdf_weight * violaPos;
  }

  if(ifprint){
    RCLCPP_INFO(logger_, "cost: %f", cost);
    RCLCPP_INFO(logger_, "cost big path dis: %f", cost_bp);
    RCLCPP_INFO(logger_, "cost final p: %f", cost_final_p);
    RCLCPP_INFO(logger_, "cost moment: %f", cost_moment);
  } 

  for(int i=0; i<TrajNum_; i++){
    partialGradByCoeffs_.block<6,1>(i*6, 1) += VecSingleXGradCS[i] * VecCoeffChainX.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_);
    partialGradByCoeffs_.block<6,1>(i*6, 0) += VecSingleXGradCTheta[i] * VecCoeffChainX.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_);
    partialGradByCoeffs_.block<6,1>(i*6, 1) += VecSingleYGradCS[i] * VecCoeffChainY.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_);
    partialGradByCoeffs_.block<6,1>(i*6, 0) += VecSingleYGradCTheta[i] * VecCoeffChainY.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_);
    partialGradByTimes_(i) += (VecSingleXGradT[i].cwiseProduct(VecCoeffChainX.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_))).sum();
    partialGradByTimes_(i) += (VecSingleYGradT[i].cwiseProduct(VecCoeffChainY.block(i*(SamNumEachPart_+1),0,SamNumEachPart_+1,1).cwiseProduct(IntegralChainCoeff_))).sum();
  }
}

}; // namespace minco_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_smoother::MincoSmoother, nav2_core::Smoother)