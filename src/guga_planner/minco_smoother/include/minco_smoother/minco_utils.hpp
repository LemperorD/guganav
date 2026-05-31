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

#ifndef NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_
#define NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_

#include <cmath>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace minco_smoother
{

struct PenaltyWeights{
    double time_weight;
    double time_weight_backup_for_replan;
    double acc_weight;
    double domega_weight;
    double collision_weight;
    double moment_weight;
    double mean_time_weight;
    double cen_acc_weight;
};

// For trajectory pre-processing
struct PathpenaltyWeights{
    double time_weight;
    double bigpath_sdf_weight;
    double mean_time_weight;
    double moment_weight;
    double acc_weight;
    double domega_weight;
};

// For trajectory pre-processing
struct PathLbfgsParams{
    lbfgs::lbfgs_parameter_t path_lbfgs_params;
    double normal_past;
    double shot_path_past;
    double shot_path_horizon;
};

inline void normalizeAngle(double from, double & to){
  while (to - from > M_PI) to -= 2.0 * M_PI;
  while (to - from < -M_PI) to += 2.0 * M_PI;
}

// Use trapezoidal velocity profile to get the total time of the path
inline double evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len; 
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  // if(endV>max_vel_){
  if(endV>maxV){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  if(length>=critical_len){
    return (maxV-startV)/maxA+(maxV-endV)/maxA+(length-critical_len)/maxV;
  }
  else{
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*length));
    return (tmpv-startV)/maxA + (tmpv-endV)/maxA;
  }
}

// Use trapezoidal velocity profile to get the distance at the curt timestamp
inline double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA){
  // std::cout<<"curt: "<<curt<<"  locallength: "<<locallength<<"  localtime: "<<localtime<<"  startV: "<<startV<<"  endV: "<<endV<<"  maxV: "<<maxV<<"  maxA: "<<maxA<<std::endl;
  double critical_len; 
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>maxV){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  // Get time from trapezoidal velocity profile
  if(locallength>=critical_len){// If locallength is greater than critical_len, accelerate to max speed and then decelerate
    double t1 = (maxV-startV)/maxA;
    double t2 = t1+(locallength-critical_len)/maxV;
    if(curt<=t1){
      return startV*curt + 0.5*maxA*pow(curt,2);
    }
    else if(curt<=t2){
      return startV*t1 + 0.5*maxA*pow(t1,2)+(curt-t1)*maxV;
    }
    else{
      return startV*t1 + 0.5*maxA*pow(t1,2) + (t2-t1)*maxV + maxV*(curt-t2)-0.5*maxA*pow(curt-t2,2);
    }
  }
  else{// If locallength is less than critical_len, decelerate without accelerating to max speed
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    double tmpt = (tmpv-startV)/maxA;
    if(curt<=tmpt){
      return startV*curt+0.5*maxA*pow(curt,2);
    }
    else{
      return startV*tmpt+0.5*maxA*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*maxA*pow(curt-tmpt,2);
    }
  }
}

template <typename EIGENVEC>
inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT){
    const int sizeT = RT.size();
    VT.resize(sizeT);
    for (int i = 0; i < sizeT; ++i){
        VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                            : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
}

template <typename EIGENVEC>
inline void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT){
    const int sizeTau = VT.size();
    RT.resize(sizeTau);
    for (int i = 0; i < sizeTau; ++i){
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
}

inline void positiveSmoothedL1(const double smoothEps, const double &x, double &f, double &df){
  const double pe = smoothEps;
  const double half = 0.5 * pe;
  const double f3c = 1.0 / (pe * pe);
  const double f4c = -0.5 * f3c / pe;
  const double d2c = 3.0 * f3c;
  const double d3c = 4.0 * f4c;

  if (x < pe){
    f = (f4c * x + f3c) * x * x * x;
    df = (d3c * x + d2c) * x * x;
  }
  else{
    f = x - half;
    df = 1.0;
  }
  return;
}

template <typename EIGENVEC>
inline void backwardGradT(const Eigen::VectorXd &tau,
                          const Eigen::VectorXd &gradT,
                          EIGENVEC &gradTau){
  const int sizetau = tau.size();
  gradTau.resize(sizetau);
  double gradrt2vt;
  for (int i = 0; i < sizetau; i++){
    if(tau(i)>0){
      gradrt2vt = tau(i)+1.0;
    }
    else{
      double denSqrt = (0.5*tau(i)-1.0)*tau(i)+1.0;
      gradrt2vt = (1.0-tau(i))/(denSqrt*denSqrt);
    }
    gradTau(i) = gradT(i) * gradrt2vt;
  }
  return;
}

bool isStateValid(const Eigen::Vector2d & pos, const nav2_costmap_2d::Costmap2D* costmap)
{
  unsigned int mx, my;
  if (!costmap->worldToMap(pos.x(), pos.y(), mx, my)) {
    return false;
  }

  unsigned char cost = costmap->getCost(mx, my);

  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

void mincoPathPub(const Trajectory<7, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta,
                  const int &sparseResolution_, const bool &if_standard_diff_, const Eigen::Vector3d &ICR_,
                  nav_msgs::msg::Path &path){
  double ini_x = start_state_XYTheta.x();
  double ini_y = start_state_XYTheta.y();

  double s1;
  int sparseResolution = sparseResolution_ * 10;
  int SamNumEachPart = 2 * sparseResolution;
  double sumT = 0.0;

  int TrajNum = final_traj.getPieceNum();
  Eigen::VectorXd pieceTime = final_traj.getDurations();

  std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
  std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
  std::vector<Eigen::VectorXd> VecYaw(TrajNum);
  std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
  VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

  for(int i=0; i<TrajNum; i++){
    double step = pieceTime[i] / sparseResolution;
    double halfstep = step / 2.0;
    double CoeffIntegral = pieceTime[i] / sparseResolution / 6.0;

    Eigen::VectorXd IntegralX(sparseResolution);IntegralX.setZero();
    Eigen::VectorXd IntegralY(sparseResolution);IntegralY.setZero();
    Eigen::VectorXd Yaw(sparseResolution);Yaw.setZero();
    s1 = 0.0;
    for(int j=0; j<=SamNumEachPart; j++){
      if(j%2 == 0){
        Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
        Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
        s1 += halfstep;

        double cosyaw = cos(currPos.x()), sinyaw = sin(currPos.x());
        if(if_standard_diff_){
          if(j!=0){
            IntegralX[j/2-1] += CoeffIntegral * currVel.y() * cos(currPos.x());
            IntegralY[j/2-1] += CoeffIntegral * currVel.y() * sin(currPos.x());
            Yaw[j/2-1] = currPos.x();
          }
          if(j!=SamNumEachPart){
            IntegralX[j/2] += CoeffIntegral * currVel.y() * cos(currPos.x());
            IntegralY[j/2] += CoeffIntegral * currVel.y() * sin(currPos.x());
          }
        }
        else{
          if(j!=0){
            IntegralX[j/2-1] += CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
            IntegralY[j/2-1] += CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
            Yaw[j/2-1] = currPos.x();
          }
          if(j!=SamNumEachPart){
            IntegralX[j/2] += CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
            IntegralY[j/2] += CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
          }
        }
      }
      else{
        Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
        Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
        s1 += halfstep;
        double cosyaw = cos(currPos.x()), sinyaw = sin(currPos.x());
        if(if_standard_diff_){
          IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
        }
        else{
          IntegralX[j/2] += 4 * CoeffIntegral * (currVel.y() * cosyaw + currVel.x() * ICR_.z() * sinyaw);
          IntegralY[j/2] += 4 * CoeffIntegral * (currVel.y() * sinyaw - currVel.x() * ICR_.z() * cosyaw);
        }
      }
    }
    VecIntegralX[i] = IntegralX;
    VecIntegralY[i] = IntegralY;
    VecYaw[i] = Yaw;
    sumT += pieceTime[i];
  }

  path.header.frame_id = "world";
  rclcpp::Clock clock;
  path.header.stamp = clock.now();
  Eigen::Vector2d pos(ini_x, ini_y);
  for(u_int i=0; i<VecIntegralX.size(); i++){
    for(u_int j=0; j<VecIntegralX[i].size(); j++){
      pos.x() += VecIntegralX[i][j];
      pos.y() += VecIntegralY[i][j];
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.header.stamp = clock.now();
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = 0.15;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, VecYaw[i][j]);
      pose.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(pose);
    }
  }
}

}

#endif // NAV2_SMOOTHER__MINCO_SMOOTHER_UTIL_HPP_