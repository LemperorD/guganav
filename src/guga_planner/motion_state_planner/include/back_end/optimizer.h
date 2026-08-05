/**
 * ================================================================
 * optimizer.h — MSPlanner: 基于 MINCO S3NU 的轨迹优化器
 * ================================================================
 *
 * MSPlanner (MINCO S3NU Planner) 是一个两阶段轨迹优化器：
 *
 * 阶段一（预处理 / path similarity）：
 *   costFunctionCallbackPath + attachPenaltyFunctionalPath
 *   目标：使优化后的路径形状尽量接近前端给出的初始路径
 *   约束：驱动轮扭矩多边形、加速度、角加速度
 *   惩罚：中间路点与初始路径点的平方距离
 *
 * 阶段二（正式优化）：
 *   costFunctionCallback + attachPenaltyFunctional
 *   目标：min 时间 + min 碰撞 + min 运动学违反
 *   约束：速度、加速度、角速度、角加速度、扭矩多边形、
 *         向心加速度、碰撞（SDF 值）、终端位置等式
 *   使用增广拉格朗日方法 (ALM) 迭代满足终端 (x,y) 约束
 *
 * 轨迹表示：
 *   sigma(t) = [theta(t), s(t)]^T — MINCO S3NU 的输出
 *   theta(t): 偏航角（5 次多项式）
 *   s(t):     弧长参数 （5 次多项式）
 *   世界坐标 (x,y) 通过 Simpson 积分从 sigma 重建
 *
 * 运动学模型：
 *   - ICR (Instantaneous Center of Rotation) 模型: 默认
 *     dx/dt = ds/dt * cos(theta) + dtheta/dt * x_ICR * sin(theta)
 *     dy/dt = ds/dt * sin(theta) - dtheta/dt * x_ICR * cos(theta)
 *   - 标准差速模型: if_standard_diff_ = true
 *     dx/dt = ds/dt * cos(theta)
 *     dy/dt = ds/dt * sin(theta)
 *
 * 优化变量编码：
 *   x = [P(2,N-1), finState_S, tau(1,N)]^T
 *   其中 tau = VirtualT2RealT(T) 将时间段映射为无约束变量
 *
 * ALM 等式约束：
 *   h(x) = IntegralFinalXY(x) - targetXY = 0
 *   增广拉格朗日: L(x,lambda) = f(x) + lambda^T h(x) + rho/2 * ||h(x)||^2
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include "ros/publisher.h"
#include "tf/transform_datatypes.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include "plan_env/sdf_map.h"
#include "front_end/traj_representation.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "gcopter/trajectory.hpp"
#include "gcopter/minco.hpp"
#include "gcopter/lbfgs.hpp"


/**
 * 运动学约束配置结构体
 * ======================
 * 从 ROS 参数服务器读取各运动学约束的上界
 * if_directly_constrain_v_omega_: true 时直接约束 v 和 omega（独立约束）
 *                                false 时使用驱动轮扭矩多边形约束（耦合约束）
 */
struct Config
    double max_vel_;
    double min_vel_;
    double max_acc_;
    double max_omega_;
    double max_domega_;
    double max_centripetal_acc_;

    bool if_directly_constrain_v_omega_;
    
    Config(const ros::NodeHandle &nh_)
    {
        nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
        nh_.param<double>(ros::this_node::getName()+ "/min_vel",min_vel_,-5);
        nh_.param<double>(ros::this_node::getName()+ "/max_acc",max_acc_,5);
        nh_.param<double>(ros::this_node::getName()+ "/max_domega",max_domega_,50);
        nh_.param<double>(ros::this_node::getName()+ "/max_centripetal_acc",max_centripetal_acc_,10000);
        nh_.param<double>(ros::this_node::getName()+ "/max_omega",max_omega_,1);
        nh_.param<bool>(ros::this_node::getName()+ "/if_directly_constrain_v_omega", if_directly_constrain_v_omega_, false);
    }
};

/**
 * 正式优化阶段的惩罚权重
 * ========================
 * - time_weight: 总时间最小化权重
 * - acc_weight: 弧长加速度 (d^2 s / dt^2) 约束惩罚
 * - domega_weight: 角加速度 (d^2 theta / dt^2) 约束惩罚
 * - collision_weight: SDF 碰撞约束惩罚
 * - moment_weight: 扭矩/速度-角速度多边形约束惩罚
 * - mean_time_weight: 段时长均衡约束惩罚
 * - cen_acc_weight: 向心加速度 (omega^2 * v) 约束惩罚
 */
struct PenaltyWeights{

// For trajectory pre-processing
/**
 * 预处理阶段的惩罚权重
 * ======================
 * 相比正式优化，预处理只需关注：
 * - bigpath_sdf_weight: 路径相似性（当前积分位置与初始路点距离）
 * - moment_weight: 扭矩约束
 * - acc_weight / domega_weight: 加速度/角加速度约束
 * - mean_time_weight: 段时长均衡
 *
 * 预处理不包含碰撞和向心加速度约束
 */
struct PathpenaltyWeights{
    double time_weight;
    double bigpath_sdf_weight;
    double mean_time_weight;
    double moment_weight;
    double acc_weight;
    double domega_weight;
};

// For trajectory pre-processing
/**
 * 预处理阶段的 L-BFGS 参数
 * ==========================
 * - normal_past: 正常路径使用的前后比较迭代数 (delta-based convergence)
 * - shot_path_past: 短路径使用的前后比较迭代数（路径太短时放宽收敛）
 * - shot_path_horizon: 判定"短路径"的弧长阈值
 */
struct PathLbfgsParams{
    lbfgs::lbfgs_parameter_t path_lbfgs_params;
    double normal_past;
    double shot_path_past;
    double shot_path_horizon;
};


/**
 * MSPlanner — 基于 MINCO S3NU 的两阶段轨迹优化器
 * ===============================================
 *
 * 核心流程：
 *   1. minco_plan() 接收前端路径
 *   2. get_state() 提取初始状态（中间点、时间段、始末条件）
 *   3. optimizer() 执行两阶段优化：
 *      a. 预处理：costFunctionCallbackPath，约束路径形状接近初始路径
 *      b. 正式优化：costFunctionCallback，ALM 循环迭代
 *   4. check_final_collision() 碰撞检测，必要时重新优化
 *
 * 优化变量编码（被 L-BFGS 优化的向量 x）：
 *   [P(2,N-1), finState_S, tau(1,N)] 维度 = 3N - 1
 *   其中 P 是中间路点矩阵 (2 × N-1)
 *   finState_S 是放松的终端弧长状态
 *   tau 是从真实时间段 T 经 RealT2VirtualT 变换的虚拟时间
 *
 * 梯度传播链：
 *   cost function -> partialGradByCoeffs/Times
 *   -> MINCO propogateArcYawLenghGrad -> gradByPoints/Times
 *   -> backwardGradT -> 优化变量的梯度
 */
class MSPlanner
{
private:
    Config config_;
    ros::NodeHandle nh_;
    std::shared_ptr<SDFmap> map_;
    
    ros::Publisher mincoinitPath;
    ros::Publisher pathmincoinitPath;
    ros::Publisher CollisionpointPub;
    ros::Publisher processmincoinitPath;

    ros::Publisher mincoinitPoint;
    ros::Publisher pathmincoinitPoint;
    ros::Publisher innerinitpositionsPoint;

    ros::Publisher recordTextPub;

    // optimizer parameters
    double mean_time_lowBound_;
    double mean_time_uppBound_;
    double smoothEps;// for smoothL1
    PenaltyWeights penaltyWt;
    Eigen::Vector2d energyWeights;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    
    double finalMinSafeDis;
    int finalSafeDisCheckNum;
    int safeReplanMaxTime;

    Eigen::Vector3d iniStateXYTheta;
    Eigen::Vector3d finStateXYTheta;

    Eigen::Vector3d final_initStateXYTheta_;
    Eigen::Vector3d final_finStateXYTheta_;

    Eigen::VectorXd pieceTime;
    Eigen::MatrixXd Innerpoints;
    Eigen::MatrixXd iniState;
    Eigen::MatrixXd finState;
    // trajectory segments number
    int TrajNum;
    // if the traj is cutted
    bool ifCutTraj_;

    std::vector<Eigen::Vector3d> inner_init_positions;

    Eigen::MatrixXd finalInnerpoints;
    Eigen::VectorXd finalpieceTime;

    minco::MINCO_S3NU Minco;
    std::vector<Eigen::Vector3d> statelist;

    PathLbfgsParams path_lbfgs_params_;
    PathpenaltyWeights PathpenaltyWt;
 

    // sampling parameters
    int sparseResolution_;
    int sparseResolution_6_;
    double timeResolution_;
    int mintrajNum_;

    int iter_num_;
    // store the gradient of the cost function
    Eigen::Matrix2Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX2d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;
    Eigen::Vector2d gradByTailStateS;
    Eigen::Vector2d FinalIntegralXYError;
    // for ALM
    Eigen::Vector2d FinalIntegralXYError_;
    // for debug, record the collision points
    std::vector<Eigen::Vector2d> collision_point;
    std::vector<Eigen::Vector2d> collision_point_;

    // unchanged auxiliary parameters in the loop
    int SamNumEachPart;
    // Simpson integration coefficients for each sampling point
    Eigen::VectorXd IntegralChainCoeff;

    // checkpoints for collision check
    std::vector<Eigen::Vector2d> check_point;
    double safeDis_, safeDis;

    // Whether to perform visualization
    bool ifprint = false;

    // Augmented Lagrangian
    Eigen::VectorXd init_EqualLambda_, init_EqualRho_, EqualRhoMax_, EqualGamma_;
    Eigen::VectorXd EqualLambda, EqualRho;
    Eigen::VectorXd EqualTolerance_;

    Eigen::VectorXd Cut_init_EqualLambda_, Cut_init_EqualRho_, Cut_EqualRhoMax_, Cut_EqualGamma_;
    Eigen::VectorXd Cut_EqualLambda, Cut_EqualRho;
    Eigen::VectorXd Cut_EqualTolerance_;

    bool hrz_limited_;
    double hrz_laser_range_dgr_;

    // Trajectory prediction resolution for get_the_predicted_state
    double trajPredictResolution_;

    bool if_visual_optimization_ = false;

public:

    // Results
    Trajectory<5, 2> final_traj_;
    // Results before collision check
    Trajectory<5, 2> optimizer_traj_;
    // Results before trajectory pre-processing
    Trajectory<5, 2> init_final_traj_;

    Eigen::Vector3d ICR_;
    bool if_standard_diff_;

    MSPlanner(const Config &conf, ros::NodeHandle &nh, std::shared_ptr<SDFmap> map);

    // Main function of the optimizer
    bool minco_plan(const FlatTrajData &flat_traj);
    // Obtain the initial state for planning
    bool get_state(const FlatTrajData &flat_traj);
    // Optimization
    bool optimizer();
    // Result check: whether a collision occurred
    bool check_final_collision(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta);

    template <typename EIGENVEC>
    inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    inline void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

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

    inline void positiveSmoothedL1(const double &x, double &f, double &df);

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                    const Eigen::VectorXd &gradT,
                                    EIGENVEC &gradTau);
    
    static double costFunctionCallbackPath(void *ptr,
                                           const Eigen::VectorXd &x,
                                           Eigen::VectorXd &g);

    void attachPenaltyFunctionalPath(double &cost);
    void mincoPathPub(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta, const ros::Publisher &publisher);
    void mincoPointPub(const Trajectory<5,2> &final_traj, const Eigen::Vector3d &start_state_XYTheta, const ros::Publisher &publisher, const Eigen::Vector3d &color);

    void Collision_point_Pub();

    Eigen::MatrixXd get_current_iniState(){
        return iniState;
    }
    Eigen::MatrixXd get_current_finState(){
        return finState;
    }
    Eigen::MatrixXd get_current_Innerpoints(){
        return finalInnerpoints;
    }
    Eigen::VectorXd get_current_finalpieceTime(){
        return finalpieceTime;
    }

    Eigen::Vector3d get_current_iniStateXYTheta(){
        return iniStateXYTheta;
    }

    void get_the_predicted_state(const double& time, Eigen::Vector3d& XYTheta, Eigen::Vector3d& VAJ, Eigen::Vector3d& OAJ);

    std::vector<Eigen::Vector3d> get_the_predicted_state_and_path(const double &start_time, const double &time, 
                                                                  const Eigen::Vector3d &start_XYTheta, 
                                                                  Eigen::Vector3d &XYTheta, bool &if_forward);

    inline double normlize_angle(double angle);

    template<typename T>
    bool readParam(const std::string &path, T &param){
        if(!nh_.hasParam(path)){
            ROS_ERROR("ERROR!! cannot read param %s", path.c_str());
            return false;
        }
        nh_.getParam(path, param);
        return true;
    }

    template<typename T>
    bool readParam(const std::string &path, T &param, const T &Default){
        if(!nh_.hasParam(path)){
            param = Default;
            ROS_ERROR("ERROR!! cannot read param %s", path.c_str());
            return false;
        }
        nh_.getParam(path, param);
        return true;
    }

    void pub_inner_init_positions(const std::vector<Eigen::Vector3d> &inner_init_positions);
};



#endif