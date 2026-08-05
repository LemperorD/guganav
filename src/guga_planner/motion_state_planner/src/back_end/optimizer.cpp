/**
 * ================================================================
 * optimizer.cpp — MSPlanner 优化器实现
 * ================================================================
 *
 * 本文件实现了基于 MINCO S3NU 轨迹参数化的两阶段轨迹优化器。
 *
 * 核心算法流程：
 *
 * 阶段一 (预处理 — 路径形状约束):
 *   costFunctionCallbackPath() + attachPenaltyFunctionalPath()
 *   1. 用 Simpson 积分从 sigma(theta,s) 重建世界坐标 (x,y)
 *   2. 惩罚中间积分位置与初始前端路点的平方偏差（保持路径形状）
 *   3. 惩罚加速度、角加速度、扭矩多边形违反
 *
 * 阶段二 (正式优化 — 完整约束):
 *   costFunctionCallback() + attachPenaltyFunctional()
 *   1. Simpson 积分重建 (x,y) 位置
 *   2. 运动学约束惩罚（加速度、角加速度、扭矩多边形、向心加速度）
 *   3. 碰撞约束惩罚（对每个 check_point 评估 SDF 双线性插值）
 *   4. 终端 (x,y) 等式约束（ALM 增广拉格朗日方法）
 *   5. 段时长均衡惩罚
 *
 * 运动学模型（可切换）:
 *   ICR 模型:   dx = ds*cosθ + dθ*ICR_xv*sinθ
 *               dy = ds*sinθ - dθ*ICR_xv*cosθ
 *   标准差速:   dx = ds*cosθ
 *               dy = ds*sinθ
 *
 * Simpson 积分:
 *   每段轨迹用 sparseResolution_ 个采样点，
 *   每个子区间用 1/6*(f0 + 4f1 + f2) 格式
 *
 * 虚拟时间映射 (RealT2VirtualT / VirtualT2RealT):
 *   将 [0, ∞) 的真实时间段映射到 (-∞, ∞) 的虚拟变量，
 *   实现无约束优化中的非负时间保障
 *
 * 增广拉格朗日方法 (ALM):
 *   终端位置约束 h(x) = IntegralFinalXY - targetXY = 0
 *   增广拉格朗日: L = f(x) + λ^T h + (ρ/2)||h||^2
 *   更新规则: λ_{k+1} = λ_k + ρ * h(x_k)
 *            ρ_{k+1} = min((1+γ)*ρ_k, ρ_max)
 *   收敛条件: ||h(x)|| < tolerance
 *
 * 碰撞检测:
 *   使用 SDF 地图的双线性插值 (getDistWithGradBilinear)
 *   对多个车身 check_point 检查，safeDis 为安全距离
 */

// MSPlanner 构造函数
// ==================
// 初始化：
// 1. ROS 发布者（用于可视化各种轨迹和调试信息）
// 2. 从参数服务器读取所有优化参数
// 3. 初始化 Simpson 积分系数 IntegralChainCoeff
// 4. 读取碰撞检测点 check_point 列表
// 5. 读取 ICR 模型参数
#include "back_end/optimizer.hpp"

MSPlanner::MSPlanner(const Config &conf, ros::NodeHandle &nh, std::shared_ptr<SDFmap> map):config_(conf){
    nh_ = nh;
    map_ = map;
    mincoinitPath = nh_.advertise<nav_msgs::Path>("/visualizer/mincoinitPath",10);
    pathmincoinitPath = nh_.advertise<nav_msgs::Path>("/visualizer/pathmincoinitPath",10);
    CollisionpointPub = nh_.advertise<sensor_msgs::PointCloud2>("/visualizer/CollisionpointPub",10);
    processmincoinitPath = nh_.advertise<nav_msgs::Path>("/visualizer/mincoPath",10);

    mincoinitPoint = nh_.advertise<visualization_msgs::Marker>("/visualizer/mincoinitPoint",10);
    pathmincoinitPoint = nh_.advertise<visualization_msgs::Marker>("/visualizer/pathmincoinitPoint",10);
    innerinitpositionsPoint = nh_.advertise<visualization_msgs::Marker>("/visualizer/innerinitpositionsPoint",10);

    recordTextPub = nh_.advertise<visualization_msgs::Marker>("/planner/calculator_time",10);

    // readParam(ros::this_node::getName()+ "/path_mean_time_lowBound", path_mean_time_lowBound_);
    // readParam(ros::this_node::getName()+ "/path_mean_time_uppBound", path_mean_time_uppBound_);
    readParam(ros::this_node::getName()+ "/mean_time_lowBound", mean_time_lowBound_);
    readParam(ros::this_node::getName()+ "/mean_time_uppBound", mean_time_uppBound_);
    readParam(ros::this_node::getName()+ "/smoothingFactor", smoothEps);
    readParam(ros::this_node::getName()+ "/safeDis", safeDis_);
    safeDis = safeDis_;

    readParam(ros::this_node::getName()+ "/finalMinSafeDis", finalMinSafeDis);
    readParam(ros::this_node::getName()+ "/finalSafeDisCheckNum", finalSafeDisCheckNum);
    readParam(ros::this_node::getName()+ "/safeReplanMaxTime", safeReplanMaxTime);

    readParam(ros::this_node::getName()+ "/penaltyWeights/time_weight",penaltyWt.time_weight_backup_for_replan);
    penaltyWt.time_weight = penaltyWt.time_weight_backup_for_replan;
    readParam(ros::this_node::getName()+ "/penaltyWeights/acc_weight",penaltyWt.acc_weight);
    readParam(ros::this_node::getName()+ "/penaltyWeights/domega_weight",penaltyWt.domega_weight);
    readParam(ros::this_node::getName()+ "/penaltyWeights/collision_weight",penaltyWt.collision_weight);
    readParam(ros::this_node::getName()+ "/penaltyWeights/moment_weight",penaltyWt.moment_weight);
    readParam(ros::this_node::getName()+ "/penaltyWeights/mean_time_weight",penaltyWt.mean_time_weight);
    readParam(ros::this_node::getName()+ "/penaltyWeights/cen_acc_weight",penaltyWt.cen_acc_weight);

    readParam(ros::this_node::getName()+ "/PathpenaltyWeights/time_weight",PathpenaltyWt.time_weight);
    readParam(ros::this_node::getName()+ "/PathpenaltyWeights/bigpath_sdf_weight",PathpenaltyWt.bigpath_sdf_weight);
    readParam(ros::this_node::getName()+ "/PathpenaltyWeights/moment_weight",PathpenaltyWt.moment_weight);
    readParam(ros::this_node::getName()+ "/PathpenaltyWeights/mean_time_weight",PathpenaltyWt.mean_time_weight);
    readParam(ros::this_node::getName()+ "/PathpenaltyWeights/acc_weight",PathpenaltyWt.acc_weight);
    readParam(ros::this_node::getName()+ "/PathpenaltyWeights/domega_weight",PathpenaltyWt.domega_weight);

    std::vector<double>  tmp_vec;

    readParam(ros::this_node::getName()+ "/energyWeights",tmp_vec);
    for(u_int i=0; i<tmp_vec.size(); i++){
        energyWeights[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/EqualLambda",tmp_vec);
    init_EqualLambda_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        init_EqualLambda_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/EqualRho",tmp_vec);
    init_EqualRho_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        init_EqualRho_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/EqualRhoMax",tmp_vec);
    EqualRhoMax_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        EqualRhoMax_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/EqualGamma",tmp_vec);
    EqualGamma_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        EqualGamma_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/EqualTolerance",tmp_vec);
    EqualTolerance_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        EqualTolerance_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/CutEqualLambda",tmp_vec);
    Cut_init_EqualLambda_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_init_EqualLambda_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/CutEqualRho",tmp_vec);
    Cut_init_EqualRho_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_init_EqualRho_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/CutEqualRhoMax",tmp_vec);
    Cut_EqualRhoMax_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_EqualRhoMax_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/CutEqualGamma",tmp_vec);
    Cut_EqualGamma_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_EqualGamma_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/CutEqualTolerance",tmp_vec);
    Cut_EqualTolerance_.resize(tmp_vec.size());
    for(u_int i=0; i<tmp_vec.size(); i++){
        Cut_EqualTolerance_[i] = tmp_vec[i];
    }

    readParam(ros::this_node::getName()+ "/path_lbfgs_params/mem_size", path_lbfgs_params_.path_lbfgs_params.mem_size);
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/past", path_lbfgs_params_.normal_past);
    path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.normal_past;
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/g_epsilon", path_lbfgs_params_.path_lbfgs_params.g_epsilon);
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/min_step", path_lbfgs_params_.path_lbfgs_params.min_step);
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/delta", path_lbfgs_params_.path_lbfgs_params.delta);
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/max_iterations", path_lbfgs_params_.path_lbfgs_params.max_iterations);
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/shot_path_past", path_lbfgs_params_.shot_path_past);
    readParam(ros::this_node::getName()+ "/path_lbfgs_params/shot_path_horizon", path_lbfgs_params_.shot_path_horizon);

    readParam(ros::this_node::getName()+ "/lbfgs_params/mem_size", lbfgs_params_.mem_size);
    readParam(ros::this_node::getName()+ "/lbfgs_params/past", lbfgs_params_.past);
    readParam(ros::this_node::getName()+ "/lbfgs_params/g_epsilon", lbfgs_params_.g_epsilon);
    readParam(ros::this_node::getName()+ "/lbfgs_params/min_step", lbfgs_params_.min_step);
    readParam(ros::this_node::getName()+ "/lbfgs_params/delta", lbfgs_params_.delta);
    readParam(ros::this_node::getName()+ "/lbfgs_params/max_iterations", lbfgs_params_.max_iterations);

    readParam(ros::this_node::getName()+ "/sparseResolution",sparseResolution_);
    readParam(ros::this_node::getName() + "/timeResolution",timeResolution_);
    readParam(ros::this_node::getName() + "/mintrajNum",mintrajNum_);

    readParam(ros::this_node::getName() + "/trajPredictResolution",trajPredictResolution_);

    readParam(ros::this_node::getName()+ "/if_visual_optimization", if_visual_optimization_, false);

    readParam(ros::this_node::getName()+ "/hrz_limited", hrz_limited_, false);
    if(hrz_limited_)  readParam(ros::this_node::getName()+ "/hrz_laser_range_dgr", hrz_laser_range_dgr_);

    // 初始化 Simpson 积分系数
    // SamNumEachPart = 2 * sparseResolution_
    // 对每一小段，系数为 [1, 4, 1] 对应三个采样点（端点+中点）
    // IntegralChainCoeff 用于梯度反向传播时的链式法则
    SamNumEachPart = 2 * sparseResolution_;
    sparseResolution_6_ = sparseResolution_ * 6;
    IntegralChainCoeff.resize(SamNumEachPart + 1);
    IntegralChainCoeff.setZero();
    for(int i=0; i<sparseResolution_; i++){
        IntegralChainCoeff.block(2*i,0,3,1) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }


    check_point.clear();
    readParam(ros::this_node::getName()+ "/checkpoint",tmp_vec);
    for(u_int i=0; i<tmp_vec.size()/2; i++){
        check_point.emplace_back(tmp_vec[i*2], tmp_vec[i*2+1]);
    }

    readParam(ros::this_node::getName() + "/ICR_yl", ICR_.x());
    readParam(ros::this_node::getName() + "/ICR_yr", ICR_.y());
    readParam(ros::this_node::getName() + "/ICR_xv", ICR_.z());

    readParam(ros::this_node::getName() + "/if_standard_diff", if_standard_diff_);
}

/**
 * minco_plan — 主优化入口函数
 * ============================
 * 流程：
 * 1. 从 flat_traj 获取初始状态 (get_state)
 * 2. 执行两阶段优化 (optimizer)
 * 3. 最终碰撞检测 (check_final_collision)
 * 4. 如存在碰撞，降低时间权重 (penaltyWt.time_weight *= 0.75) 重新优化
 * 5. 最多重试 safeReplanMaxTime 次
 *
 * 返回：true 表示成功生成无碰撞轨迹
 */
bool MSPlanner::minco_plan(const FlatTrajData &flat_traj){

    ros::Time minco_start = ros::Time::now();
    ros::Time current = ros::Time::now();
    bool final_collision = false;
    int replan_num_for_coll = 0;

    double start_safe_dis = map_->getDistanceReal(flat_traj.start_state_XYTheta.head(2))*0.85;
    safeDis = std::min(start_safe_dis, safeDis_); // 动态调整安全距离：取预设值和当前SDF值的85%中的较小者
    
    for(; replan_num_for_coll < safeReplanMaxTime; replan_num_for_coll++){

        if(get_state(flat_traj))
            ROS_INFO("\033[40;36m get_state time:%f \033[0m", (ros::Time::now()-current).toSec());
        else
            return false;
        current = ros::Time::now();
        if(optimizer())
            ROS_INFO("\033[41;37m minco optimizer time:%f \033[0m", (ros::Time::now()-current).toSec());
        else
            return false;

        Minco.getTrajectory(optimizer_traj_);
        final_collision = check_final_collision(optimizer_traj_, iniStateXYTheta);
        if(final_collision){
            penaltyWt.time_weight *= 0.75;  // 存在碰撞：降低时间权重，让轨迹更紧凑
        }
        else{
            break;  // 无碰撞：退出重试循环
        }
    }
    Collision_point_Pub();  // 发布碰撞点用于可视化调试
    penaltyWt.time_weight = penaltyWt.time_weight_backup_for_replan;  // 恢复原始时间权重
    safeDis = safeDis_;  // 恢复原始安全距离
    if(replan_num_for_coll == safeReplanMaxTime){
        ROS_ERROR("\n\n\n final traj Collision!!!!!!!!!!!\n\n\n\n");
        return false;
    }

    final_traj_ = optimizer_traj_;
    final_initStateXYTheta_ = iniStateXYTheta;
    final_finStateXYTheta_ = finStateXYTheta;
    Collision_point_Pub();

    ROS_INFO("-------------------------------------------------------------------------------------------------------------------------------");
    
    ROS_INFO("\033[41;37m all of back_end time:%f, with optimizer %d times. \033[0m", (ros::Time::now()-minco_start).toSec(), replan_num_for_coll+1);

    return true;
}

/**
 * get_state — 从前端轨迹数据提取初始状态
 * ========================================
 * flat_traj 包含：
 * - UnOccupied_traj_pts: 无碰撞中间路点（世界坐标）
 * - start_state / final_state: 始末 (theta, s) 及其导数
 * - UnOccupied_initT: 每段初始时间
 * - start_state_XYTheta / final_state_XYTheta: 世界坐标始末位姿
 */
bool MSPlanner::get_state(const FlatTrajData &flat_traj){
    ifCutTraj_ = flat_traj.if_cut;

    TrajNum = flat_traj.UnOccupied_traj_pts.size()+1;

    Innerpoints.resize(2,TrajNum-1);
    for(u_int i=0; i<flat_traj.UnOccupied_traj_pts.size(); i++){
        Innerpoints.col(i) = flat_traj.UnOccupied_traj_pts[i].head(2);
    }

    inner_init_positions = flat_traj.UnOccupied_positions;
    inner_init_positions.push_back(flat_traj.final_state_XYTheta);

    iniState = flat_traj.start_state;
    finState = flat_traj.final_state;

    pieceTime.resize(TrajNum); pieceTime.setOnes();
    pieceTime *= flat_traj.UnOccupied_initT;

    iniStateXYTheta = flat_traj.start_state_XYTheta;
    finStateXYTheta = flat_traj.final_state_XYTheta;

    pub_inner_init_positions(inner_init_positions);
    
    return true;
}

/**
 * optimizer — 两阶段轨迹优化核心函数
 * ===================================
 *
 * 优化变量 x 的编码：
 *   [P_00, P_10, P_01, P_11, ..., P_{N-2,0}, P_{N-2,1}, finState_S, tau_0, ..., tau_{N-1}]
 *   其中 P_{i,j} 是第 i 个中间路点的第 j 维 (theta, s)
 *   finState_S 是放松的终端弧长
 *   tau = VirtualT2RealT(T) 是虚拟时间变量（无约束）
 *
 * 第一阶段：预处理
 *   - 使用 L-BFGS 求解器优化
 *   - 回调 costFunctionCallbackPath → attachPenaltyFunctionalPath
 *   - 目标：路径形状接近初始路径
 *
 * 第二阶段：正式优化
 *   - 使用 L-BFGS + ALM 循环优化
 *   - 回调 costFunctionCallback → attachPenaltyFunctional
 *   - 每次 L-BFGS 收敛后检查终端位置约束
 *   - 如果 ||h(x)|| > tolerance，更新 λ 和 ρ 重新优化
 */
bool MSPlanner::optimizer(){
    // Initialize Lagrangian
    // 根据是否被截断选择不同的 ALM 参数
    if(!ifCutTraj_){
        EqualLambda = init_EqualLambda_;
        EqualRho = init_EqualRho_;
    }
    else{
        EqualLambda = Cut_init_EqualLambda_;
        EqualRho = Cut_init_EqualRho_;
    }

    // 变量总数: 2*(N-1) + 1 + N = 3N - 1
    int variable_num_ = 3 * TrajNum - 1;
    // ROS_INFO_STREAM("iniStates: \n" << iniState);
    // ROS_INFO_STREAM("finStates: \n" << finState);
    // ROS_INFO("TrajNum: %d", TrajNum);
    Minco.setConditions(iniState, finState, TrajNum, energyWeights);

    // ROS_INFO_STREAM("init Innerpoints: \n" << Innerpoints);
    // ROS_INFO_STREAM("init pieceTime: " << pieceTime.transpose());

    Minco.setParameters(Innerpoints, pieceTime);   
    Minco.getTrajectory(init_final_traj_);
    mincoPathPub(init_final_traj_, iniStateXYTheta, mincoinitPath); 
    mincoPointPub(init_final_traj_, iniStateXYTheta, mincoinitPoint, Eigen::Vector3d(173, 127, 168));
    // 将优化变量打包成向量 x
    // memcpy 布局: [P(2,N-1), finState_S, tau(N)]
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    memcpy(x.data()+offset,Innerpoints.data(), Innerpoints.size() * sizeof(x[0]));
    offset += Innerpoints.size();
    x[offset] = finState(1,0);  // 放松的终端弧长
    ++offset;
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, pieceTime.size());
    offset += pieceTime.size();
    RealT2VirtualT(pieceTime,Vt);  // 真实时间 -> 虚拟时间（无约束变量）

    double cost;
    int result;
    Eigen::VectorXd g;
    g.resize(x.size());
    iter_num_ = 0;

    auto start = std::chrono::high_resolution_clock::now();
    // ==========================================
    // 阶段一：预处理优化（路径形状约束）
    // ==========================================
    // 如果路径太短（finState_S < shot_path_horizon），使用更宽松的收敛条件
    if (fabs(finState(1, 0)) < path_lbfgs_params_.shot_path_horizon) {
        path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.shot_path_past;
    } else {
        path_lbfgs_params_.path_lbfgs_params.past = path_lbfgs_params_.normal_past;
    }

    ifprint = false;
    result = lbfgs::lbfgs_optimize(x,
                                cost,
                                MSPlanner::costFunctionCallbackPath,
                                NULL,
                                NULL,
                                this,
                                path_lbfgs_params_.path_lbfgs_params);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Output computation time
    // 发布预处理时间文本
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
    costFunctionCallbackPath(this,x,g);
    ifprint = false;

    ROS_INFO_STREAM("Pre-processing optimizer:" << duration / 1000.0 << " ms");
    ROS_INFO("Pre-processing finish! result:%d   finalcost:%f   iter_num_:%d", result, cost, iter_num_);
    // 提取预处理结果
    offset = 0;
    Eigen::Map<Eigen::MatrixXd> PathP(x.data() + offset, 2, TrajNum - 1);  // 预处理中间点
    offset += 2 * (TrajNum - 1);
    finalInnerpoints = PathP;
    finState(1, 0) = x[offset];  // 预处理终端弧长
    ++offset;

    Eigen::Map<const Eigen::VectorXd> Patht(x.data() + offset, TrajNum);  // 预处理时间段
    offset += TrajNum;
    VirtualT2RealT(Patht, finalpieceTime);  // 虚拟时间 -> 真实时间
    Minco.setTConditions(finState);
    Minco.setParameters(finalInnerpoints, finalpieceTime);
    Minco.getTrajectory(final_traj_);
    mincoPathPub(final_traj_, iniStateXYTheta, pathmincoinitPath);
    mincoPointPub(final_traj_, iniStateXYTheta, pathmincoinitPoint, Eigen::Vector3d(114, 159, 207));

    // ROS_INFO_STREAM("Path final pieces time: " << finalpieceTime.transpose());
    // ROS_INFO_STREAM("Path final Innerpoints: \n" << finalInnerpoints);
    // ROS_INFO_STREAM("Path final finState: \n" << finState);

    // ==========================================
    // 阶段二：正式优化 (L-BFGS + ALM 迭代)
    // ==========================================
    // 外循环：ALM 更新 lambda 和 rho
    // 内循环：L-BFGS 求解当前的增广拉格朗日子问题
    ROS_INFO("-------------------------------------------------------------------optimize---------------------------------------------------------------");
    iter_num_ = 0;
    
    ros::Time current = ros::Time::now();

    while(ros::ok()){
        result = lbfgs::lbfgs_optimize(x,
                                       cost,
                                       MSPlanner::costFunctionCallback,
                                       NULL,
                                       MSPlanner::earlyExit,
                                       this,
                                       lbfgs_params_);
        if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
            result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION){
            ROS_INFO("optimizer finish! result:%d   finalcost:%f   iter_num_:%d ",result,cost,iter_num_);
        } 
        else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
            ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
        }
        else{
            ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        }
        // ALM 更新：检查等式约束 h(x) = FinalIntegralXYError
        // h(x) = 积分重建终点 - 目标终点 = [ex, ey]^T
        // 增广拉格朗日: L = f(x) + λ^T h(x) + (ρ/2) ||h(x)||^2
        // 更新：λ_{k+1} = λ_k + ρ * h(x_k)
        //       ρ_{k+1} = min((1+γ) * ρ_k, ρ_max)
        // 收敛：||h(x)|| < tolerance
        if(!ifCutTraj_){

            // ROS_INFO_STREAM("EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
            if(FinalIntegralXYError.norm() < EqualTolerance_[0]){
                break;
            }
            EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
            EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
            EqualRho[0] = std::min((1 + EqualGamma_[0]) * EqualRho[0], EqualRhoMax_[0]);
            EqualRho[1] = std::min((1 + EqualGamma_[1]) * EqualRho[1], EqualRhoMax_[1]);
        }
        else{
            // ROS_INFO_STREAM("EqualLambda: " << EqualLambda.transpose() << "  EqualRho: " << EqualRho.transpose() << "  current hx cost:" << FinalIntegralXYError.transpose() << "  XYError.norm():" << FinalIntegralXYError.norm());
            if(FinalIntegralXYError.norm() < Cut_EqualTolerance_[0]){
                break;
            }
            EqualLambda[0] += EqualRho[0] * FinalIntegralXYError.x();
            EqualLambda[1] += EqualRho[1] * FinalIntegralXYError.y();
            EqualRho[0] = std::min((1 + Cut_EqualGamma_[0]) * EqualRho[0], Cut_EqualRhoMax_[0]);
            EqualRho[1] = std::min((1 + Cut_EqualGamma_[1]) * EqualRho[1], Cut_EqualRhoMax_[1]);

        }

    }

    double mincotime = (ros::Time::now()-current).toSec();
    ROS_INFO("\033[40;36m minco optimizer time:%f \033[0m", mincotime);
    ROS_INFO_STREAM("--------------------------------------------------------------------final------------------------------------------------------");

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "minco";
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
    Eigen::Map<Eigen::MatrixXd> P(x.data()+offset, 2, TrajNum - 1);
    offset += 2 * (TrajNum - 1);
    finalInnerpoints = P;

    finState(1,0) = x[offset];
    ++offset;

    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, TrajNum);
    offset += TrajNum;
    VirtualT2RealT(t,finalpieceTime);
    Minco.setTConditions(finState);
    Minco.setParameters(finalInnerpoints, finalpieceTime);

    // std::cout<<"finalInnerpoints: \n"<<finalInnerpoints<<std::endl;
    // std::cout<<"finalpieceTime: \n"<<finalpieceTime.transpose()<<std::endl;

    ROS_INFO("\n\n optimizer finish! result:%d   finalcost:%f   iter_num_:%d\n\n ",result,cost,iter_num_);

    return true;
}

/**
 * check_final_collision — 最终碰撞检测
 * ======================================
 * 将轨迹通过 Simpson 积分重建为世界坐标 (x,y)，
 * 在每个积分点评估 SDF 双线性插值的距离值。
 * 如果任何点的 SDF 值 < finalMinSafeDis，返回 true（存在碰撞）。
 *
 * 采样的密集度为 finalSafeDisCheckNum（高于优化时的采样密度）。
 */
bool MSPlanner::check_final_collision(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta){
    double ini_x = start_state_XYTheta.x();
    double ini_y = start_state_XYTheta.y();

    double s1;
    int sparseResolution = finalSafeDisCheckNum;
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
            if(if_standard_diff_){
                if(j%2 == 0){
                    Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                    Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
                    s1 += halfstep;
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
                    Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                    Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
                    s1 += halfstep;
                    IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
                    IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
                }
            }
            else{
                Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
                Eigen::Vector2d dsigma = final_traj.getVel(s1+sumT);
                double cosyaw = cos(currPos.x());
                double sinyaw = sin(currPos.x());
                s1 += halfstep;
                if(j%2 == 0){
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2-1] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }
                    if(j!=SamNumEachPart){
                        IntegralX[j/2] += CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                        IntegralY[j/2] += CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                    }
                }
                else{
                    IntegralX[j/2] += 4.0 * CoeffIntegral * (dsigma.y() * cosyaw + dsigma.x() * ICR_.z() * sinyaw);
                    IntegralY[j/2] += 4.0 * CoeffIntegral * (dsigma.y() * sinyaw - dsigma.x() * ICR_.z() * cosyaw);
                }
            }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecYaw[i] = Yaw;
        // VecTrajFinalXY[i+1] = Eigen::Vector2d(IntegralX[IntegralX.size()-1], IntegralY[IntegralX.size()-1]);
        sumT += pieceTime[i];
    }
    double min_distance = DBL_MAX;
    Eigen::Vector2d pos(ini_x, ini_y);
    for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
            pos.x() += VecIntegralX[i][j];
            pos.y() += VecIntegralY[i][j];
            double SDFvalue = map_->getDistWithGradBilinear(pos);
            if(SDFvalue < min_distance)
                min_distance = SDFvalue;
            if(SDFvalue < finalMinSafeDis){
                ROS_INFO("SDFvalue < finalMinSafeDis!!!  min Distance: %f", min_distance);
                return true;
            }
                
        }
    }
    ROS_INFO("\033[39;35m min Distance: %f \033[0m", min_distance);
    return false;
}

/**
 * RealT2VirtualT — 真实时间到虚拟时间的映射
 * ==========================================
 * 将 [1, ∞) 的真实时间段 T 映射到虚拟变量 τ：
 *   T > 1 时：τ = sqrt(2T - 1) - 1
 *   T < 1 时：τ = 1 - sqrt(2/T - 1)
 * 映射性质：T ∈ (0, ∞) → τ ∈ (-∞, ∞)
 * T = 1 是映射的对称点：τ = 0
 *
 * 用途：将非负时间段 T 映射为无约束的优化变量 τ，
 *       使得 L-BFGS 可以在 (-∞, ∞) 上自由搜索，
 *       而保证 T = VirtualT2RealT(τ) > 0 恒成立
 */
template <typename EIGENVEC>
inline void MSPlanner::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT){
    const int sizeT = RT.size();
    VT.resize(sizeT);
    for (int i = 0; i < sizeT; ++i){
        VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                            : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
}

/**
 * VirtualT2RealT — 虚拟时间到真实时间的映射（逆变换）
 * ====================================================
 * τ > 0 时：T = (0.5τ + 1)τ + 1 = 0.5τ^2 + τ + 1
 * τ < 0 时：T = 1 / ((0.5τ - 1)τ + 1) = 1 / (0.5τ^2 - τ + 1)
 *
 * 性质：T > 0 恒成立，保证时间段的物理可行性
 */
template <typename EIGENVEC>
inline void MSPlanner::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT){
    const int sizeTau = VT.size();
    RT.resize(sizeTau);
    for (int i = 0; i < sizeTau; ++i){
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
}

/**
 * earlyExit — L-BFGS 每次迭代回调
 * ================================
 * 1. 保存当前的终端误差和碰撞点供外层 ALM 使用
 * 2. 如果开启可视化 (if_visual_optimization_)，发布中间轨迹
 * 3. 返回 0 继续优化，非零将取消优化
 */
inline int MSPlanner::earlyExit(void *instance,
                            const Eigen::VectorXd &x,
                            const Eigen::VectorXd &g,
                            const double fx,
                            const double step,
                            const int k,
                            const int ls){
    MSPlanner &obj = *(MSPlanner *)instance;
    obj.FinalIntegralXYError_ = obj.FinalIntegralXYError;
    obj.collision_point_ = obj.collision_point;
    // std::cout<<"cost: "<<fx<<std::endl;

    if(obj.if_visual_optimization_){
        ROS_INFO("fx: %f  step: %f  k: %d  ls: %d", fx, step, k, ls);
        ROS_INFO_STREAM("x: " << x.transpose());
        ROS_INFO_STREAM("g: " << g.transpose());
        ROS_INFO_STREAM("");
        int offset = 0;
        Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum - 1);
        offset += 2 * (obj.TrajNum - 1);
        obj.finalInnerpoints = P;

        obj.finState(1,0) = x[offset];
        ++offset;

        Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum);
        offset += obj.TrajNum;
        obj.VirtualT2RealT(t,obj.finalpieceTime);
        obj.Minco.setTConditions(obj.finState);
        obj.Minco.setParameters(obj.finalInnerpoints, obj.finalpieceTime);
        obj.Minco.getTrajectory(obj.optimizer_traj_);
        obj.mincoPathPub(obj.optimizer_traj_, obj.iniStateXYTheta, obj.processmincoinitPath);
        ros::Duration(0.5).sleep();
    }
    return 0;
}


/**
 * costFunctionCallback — 正式优化阶段的代价函数回调
 * ==================================================
 *
 * L-BFGS 在每次迭代中调用此函数，需要同时提供：
 *   - 代价函数值 f(x)
 *   - 梯度向量 ∇f(x) 存储在 g 中
 *
 * 工作流程：
 * 1. 从优化变量 x 中提取中间点 P、终端弧长 finState_S、虚拟时间 tau
 * 2. VirtualT2RealT：虚拟时间 → 真实时间段
 * 3. 调用 MINCO setParameters 求解多项式系数
 * 4. 获取 MINCO 能量及其对系数和时间的偏导数
 * 5. 调用 attachPenaltyFunctional 加入约束惩罚
 * 6. MINCO propogateArcYawLenghGrad：将系数/时间梯度传播到优化变量
 * 7. backwardGradT：虚拟时间的梯度通过链式法则转换
 *
 * 代价组成：
 *   cost = energy + time_weight * ΣT_i + constraint_penalties
 *
 * 梯度链：
 *   g = [∂f/∂P, ∂f/∂finState_S, ∂f/∂tau]
 *   其中 ∂f/∂tau = ∂f/∂T * ∂T/∂tau (backwardGradT)
 */
double MSPlanner::costFunctionCallback(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g){

    if(x.norm()>1e4)
        return inf;  // 防止优化变量爆炸

    MSPlanner &obj = *(MSPlanner *)ptr;
    obj.iter_num_ += 1;

    g.setZero();
    // ==========================================
    // 步骤1：将优化变量 x 映射到物理变量
    // x 布局: [P(2,N-1), finState_S, tau(N)]
    // ==========================================
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum - 1);  // 中间路点
    Eigen::Map<Eigen::MatrixXd> gradP(g.data()+offset, 2, obj.TrajNum - 1);    // 中间路点梯度
    offset += 2 * (obj.TrajNum - 1);

    double* gradTailS = g.data()+offset;  // 终端弧长梯度指针
    obj.finState(1,0) = x[offset];  // 放松的终端弧长状态
    ++offset;

    gradP.setZero();
    obj.Innerpoints = P;

    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum);  // 虚拟时间
    Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.TrajNum);    // 虚拟时间梯度
    offset += obj.TrajNum;
    obj.VirtualT2RealT(t, obj.pieceTime);  // 虚拟时间 -> 真实时间
    gradt.setZero();

    // ==========================================
    // 步骤2：计算 MINCO 多项式系数并获取能量梯度
    // ==========================================
    double cost;
    obj.Minco.setTConditions(obj.finState);
    obj.Minco.setParameters(obj.Innerpoints,obj.pieceTime);
    obj.Minco.getEnergy(cost);
    obj.Minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.Minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    if(obj.ifprint){
        ROS_INFO("Energy cost: %f", cost);
    }
    // ==========================================
    // 步骤3：加入约束惩罚（运动学 + 碰撞 + ALM 终端约束）
    // ==========================================
    obj.attachPenaltyFunctional(cost);
    if(obj.ifprint){
        ROS_INFO("attachPenaltyFunctional cost: %f", cost);
    }
    // ==========================================
    // 步骤4：梯度传播 — 从系数/时间梯度到优化变量梯度
    // propogateArcYawLenghGrad: 同时放松 theta 和 s 终端
    //   partialGradByCoeffs → gradByPoints (对中间路点)
    //   partialGradByTimes  → gradByTimes  (对时间段)
    //   → gradByTailStateS   (对终端 s 状态)
    // ==========================================
    obj.Minco.propogateArcYawLenghGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                        obj.gradByPoints, obj.gradByTimes, obj.gradByTailStateS);

    // 时间总代价：time_weight * ΣT_i
    cost += obj.penaltyWt.time_weight * obj.pieceTime.sum();
    if(obj.ifprint){
        ROS_INFO("T cost: %f", obj.penaltyWt.time_weight * obj.pieceTime.sum());
    }
    Eigen::VectorXd rhotimes;
    rhotimes.resize(obj.gradByTimes.size());
    obj.gradByTimes += obj.penaltyWt.time_weight * rhotimes.setOnes();  // ∂(ΣT_i)/∂T_i = 1

    // 终端 s 状态的梯度
    *gradTailS = obj.gradByTailStateS.y();  // 只传递 s 维度的梯度（theta 终端固定）

    gradP = obj.gradByPoints;  // 中间路点的梯度
    backwardGradT(t, obj.gradByTimes, gradt);  // 虚拟时间梯度: ∂f/∂tau = ∂f/∂T * ∂T/∂tau
    
    return cost;
}

/**
 * attachPenaltyFunctional — 正式优化阶段的约束惩罚计算
 * ======================================================
 *
 * 本函数是优化器的核心，对当前轨迹 sigma(t) = [theta(t), s(t)]
 * 施加各类约束并通过 Simpson 积分重建世界坐标 (x,y)。
 *
 * 用到的约束惩罚（全部使用 smooth L1 光滑近似）：
 *
 * 1. 加速度约束：
 *    violaAcc = (d^2 s/dt^2)^2 - max_acc^2 > 0
 *    惩罚过大的弧长加速度
 *
 * 2. 角加速度约束：
 *    violaAlp = (d^2 theta/dt^2)^2 - max_domega^2 > 0
 *    惩罚过大的角加速度
 *
 * 3. 速度/角速度约束（可选的两种模式）：
 *    模式A (if_directly_constrain_v_omega_=true):
 *      独立约束 v^2 和 omega^2，通过 moment_weight 惩罚
 *    模式B (默认):
 *       驱动轮扭矩多边形约束（对称四边形），4 条超平面
 *       violaMom = ± max_vel * dtheta/dt + max_omega * ds/dt - max_vel * max_omega > 0
 *       和对称的 min_vel 版本
 *
 * 4. 向心加速度约束（防侧滑/防侧翻）：
 *    violaCenAcc = (dtheta/dt)^2 * (ds/dt)^2 - max_centripetal_acc^2 > 0
 *
 * 5. 碰撞约束：
 *    对车身多个检测点 (check_point)，在 SDF 地图上做双线性插值
 *    violaPos = -SDF_value + safeDis > 0
 *
 * 6. 段时长均衡：
 *    惩罚单段时长偏离平均时长的上下界过多
 *
 * 7. 终端位置约束 (ALM)：
 *    增广拉格朗日: (ρ/2) * ||IntegralFinalXY - targetXY + λ/ρ||^2
 *
 * Simpson 积分（世界坐标重建）：
 *   每个时间段 [0, T_i] 被均匀划分为 sparseResolution_ 个子区间
 *   每个子区间用 3 点 Simpson: ∫ = (T/(6N)) * [f0 + 4f1 + f2]
 *   其中 f0, f2 是端点，f1 是中点
 *
 * ICR 运动学模型 (默认)：
 *   dx/dt = ds/dt * cos(theta) + dtheta/dt * ICR_z * sin(theta)
 *   dy/dt = ds/dt * sin(theta) - dtheta/dt * ICR_z * cos(theta)
 *
 * 标准差速模型 (if_standard_diff_=true)：
 *   dx/dt = ds/dt * cos(theta)
 *   dy/dt = ds/dt * sin(theta)
 *
 * 梯度链（以加速度约束为例）：
 *   cost += ω * step * smoothL1(violaAcc)
 *   ∂cost/∂ddsigma = ω * step * smoothL1'(violaAcc) * 2 * ddsigma
 *   通过 gradBeta 进一步对多项式系数 c 求导：
 *     ∂cost/∂c = ∂cost/∂ddsigma * ∂ddsigma/∂c = ∂cost/∂ddsigma * beta2
 *   其中 beta2 = [0,0,2,6t,12t^2,20t^3] 是 sigma(t) 对 c 的二次导数
 */
void MSPlanner::attachPenaltyFunctional(double &cost){
    collision_point.clear();
    double ini_x = iniStateXYTheta.x();
    double ini_y = iniStateXYTheta.y();

    // beta0-3 是多项式基函数的各阶导数在 s 处的值
    // beta0 = [1, s, s^2, s^3, s^4, s^5]  (位置基)
    // beta1 = [0, 1, 2s, 3s^2, 4s^3, 5s^4] (速度基)
    // beta2 = [0, 0, 2, 6s, 12s^2, 20s^3]  (加速度基)
    // beta3 = [0, 0, 0, 6, 24s, 60s^2]      (jerk基)
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    double IntegralAlpha, Alpha, omg, omgstep;
    
    double unoccupied_averageT;
    unoccupied_averageT = pieceTime.mean();
    
    double cost_corrb=0, cost_v=0, cost_a=0, cost_omega = 0, cost_domega=0, cost_endp=0, cost_moment=0, cost_meanT=0, cost_centripetal_acc=0;
    
    double violaAcc, violaAlp, violaPos, violaMom, violaCenAcc;
    double violaAccPena, violaAlpPena, violaPosPena, violaMomPena, violaCenAccPena;
    double violaAccPenaD, violaAlpPenaD, violaPosPenaD, violaMomPenaD, violaCenAccPenaD;
    double gradViolaAT, gradViolaDOT, gradViolaPt, gradViolaMt, gradViolaCAt;

    double violaVel, violaVelPena, violaVelPenaD;
    double violaOmega, violaOmegaPena, violaOmegaPenaD;

    Eigen::Matrix2d help_L;
    Eigen::Vector3d gradESDF;
    Eigen::Vector2d gradESDF2d;
    

    // 梯度存储变量
    // VecIntegralX/Y: 存储每个采样时刻的 Simpson 积分增量 dx, dy
    // VecSingleXGradCS/CTheta/T: 存储 x 方向积分对多项式系数、theta 系数、时间的梯度
    // 用于在约束惩罚后通过链式法则将梯度传播到多项式系数
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

    Eigen::MatrixXd SingleXGradCS(6,SamNumEachPart+1);
    Eigen::MatrixXd SingleXGradCTheta(6,SamNumEachPart+1);
    Eigen::VectorXd SingleXGradT(SamNumEachPart+1);
    Eigen::MatrixXd SingleYGradCS(6,SamNumEachPart+1);
    Eigen::MatrixXd SingleYGradCTheta(6,SamNumEachPart+1);
    Eigen::VectorXd SingleYGradT(SamNumEachPart+1);
    Eigen::VectorXd IntegralX(sparseResolution_);
    Eigen::VectorXd IntegralY(sparseResolution_);

    // Used to store the positions obtained by integration
    Eigen::VectorXd VecCoeffChainX(TrajNum*(SamNumEachPart+1));VecCoeffChainX.setZero();
    Eigen::VectorXd VecCoeffChainY(TrajNum*(SamNumEachPart+1));VecCoeffChainY.setZero();
    Eigen::Vector2d CurrentPointXY(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        // 获取第 i 段的多项式系数
        const Eigen::Matrix<double, 6, 2> &c = Minco.getCoeffs().block<6,2>(6*i, 0);
        double step = pieceTime[i] / sparseResolution_;  // 子区间步长
        double halfstep = step / 2.0;  // Simpson 积分中点步长
        double CoeffIntegral = pieceTime[i] / sparseResolution_6_;  // Simpson 积分系数 T/(6N)
        // 即 T_i / (6 * N)，其中 N = sparseResolution_
        
        IntegralX.setZero();
        IntegralY.setZero();
        
        s1 = 0.0;

        for(int j=0; j<=SamNumEachPart; j++){
            // Simpson 积分分为偶数和奇数采样点
            // j%2==0: 端点 (权重 1)，贡献给相邻的两个子区间
            // j%2==1: 中点 (权重 4)，只贡献给当前子区间
            if(j%2 == 0){
                // ==========================================
                // 偶索引采样点（子区间端点）
                // ==========================================
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                // 计算多项式基函数及其导数
                beta0 << 1.0, s1, s2, s3, s4, s5;          // 位置基
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;  // 速度基
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;     // 加速度基
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;          // jerk基
                s1 += halfstep;  // 推进到下一个采样点
                IntegralAlpha = 1.0 / SamNumEachPart * j;  // 归一化时间索引 [0,1]
                Alpha = 1.0 / sparseResolution_ * (double(j)/2);  // 实际时间比例
                omg = (j==0||j==SamNumEachPart)? 0.5:1;  // Simpson 权重系数
                omgstep = omg * step;  // 加权步长 = ω * step
                // 多项式求值: sigma = c^T * beta
                sigma = c.transpose() * beta0;      // [theta, s]
                dsigma = c.transpose() * beta1;      // [dtheta/dt, ds/dt]
                ddsigma = c.transpose() * beta2;     // [d^2theta/dt^2, d^2s/dt^2]
                dddsigma = c.transpose() * beta3;    // [d^3theta/dt^3, d^3s/dt^3]
                // cos/sin 预计算用于运动学积分
                double cosyaw = cos(sigma.x()), sinyaw = sin(sigma.x());
                

                if(if_standard_diff_){
                    if(j!=0){
                        IntegralX[j/2-1] += CoeffIntegral * dsigma.y() * cosyaw;
                        IntegralY[j/2-1] += CoeffIntegral * dsigma.y() * sinyaw;
                    }
                    if(j!=SamNumEachPart){
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
                    if(j!=SamNumEachPart){
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
                
                
                violaAcc = ddsigma.y()*ddsigma.y() - config_.max_acc_*config_.max_acc_;
                violaAlp = ddsigma.x()*ddsigma.x() - config_.max_domega_*config_.max_domega_;
                
                if(violaAcc > 0){
                    positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
                    gradViolaAT = 2.0 * Alpha * ddsigma.y() * dddsigma.y();
                    gradBeta(2,1) +=  omgstep * penaltyWt.acc_weight * violaAccPenaD * 2.0 * ddsigma.y();
                    partialGradByTimes(i) += omg * penaltyWt.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / sparseResolution_);
                    cost += omgstep * penaltyWt.acc_weight * violaAccPena;
                    cost_a += omgstep * penaltyWt.acc_weight * violaAccPena;
                }
                if(violaAlp > 0){
                    positiveSmoothedL1(violaAlp, violaAlpPena, violaAlpPenaD);
                    gradViolaDOT = 2.0 * Alpha * ddsigma.x() * dddsigma.x();
                    gradBeta(2,0) += omgstep * penaltyWt.domega_weight * violaAlpPenaD * 2.0 * ddsigma.x();
                    partialGradByTimes(i) += omg * penaltyWt.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / sparseResolution_);
                    cost += omgstep * penaltyWt.domega_weight * violaAlpPena;
                    cost_domega += omgstep * penaltyWt.domega_weight * violaAlpPena;
                }

                if(config_.if_directly_constrain_v_omega_){
                    // Directly constrain velocity and angular velocity
                    violaVel = dsigma.y() * dsigma.y() - config_.max_vel_ * config_.max_vel_;
                    if(violaVel > 0){
                        positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);
                        gradViolaPt = 2.0 * Alpha * dsigma.y()
                                        * ddsigma.y();
                        gradBeta(1,1) += omgstep * penaltyWt.moment_weight * violaVelPenaD * 2.0 * dsigma.y();
                        partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaVelPenaD * gradViolaPt * step + violaVelPena / sparseResolution_);
                        cost += omgstep * penaltyWt.moment_weight * violaVelPena;
                        cost_moment += omgstep * penaltyWt.moment_weight * violaVelPena;
                    }
                    violaOmega = dsigma.x() * dsigma.x() - config_.max_omega_ * config_.max_omega_;
                    if(violaOmega > 0){
                        positiveSmoothedL1(violaOmega, violaOmegaPena, violaOmegaPenaD);
                        gradViolaPt = 2.0 * Alpha * dsigma.x()
                                        * ddsigma.x();
                        gradBeta(1,0) += omgstep * penaltyWt.moment_weight * violaOmegaPenaD * 2.0 * dsigma.x();
                        partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaOmegaPenaD * gradViolaPt * step + violaOmegaPena / sparseResolution_);
                        cost += omgstep * penaltyWt.moment_weight * violaOmegaPena;
                        cost_moment += omgstep * penaltyWt.moment_weight * violaOmegaPena;
                    }
                }
                else{
                    // ==========================================
                    // 驱动轮扭矩多边形约束（ICR 模型的对称四边形）
                    // ==========================================
                    // 约束形式：对左右轮扭矩同时约束，
                    // 形成以 (0,0) 为中心的四边形可行域
                    // 第一组：右轮约束（omg_sym = ±1 对应左右轮）
                    //   ± max_vel * dtheta/dt + max_omega * ds/dt - max_vel * max_omega <= 0
                    // 第二组：左轮约束
                    //   ± (-min_vel) * dtheta/dt - max_omega * ds/dt + min_vel * max_omega <= 0
                    // 这里使用 smooth L1 惩罚违反量
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                        violaMom = omg_sym * config_.max_vel_ * dsigma.x() + config_.max_omega_ * dsigma.y() - config_.max_vel_ * config_.max_omega_;
                        if(violaMom > 0){
                            positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = Alpha * (omg_sym * config_.max_vel_ * ddsigma.x() + config_.max_omega_ * ddsigma.y());
                            gradBeta(1,0) += omgstep * penaltyWt.moment_weight * violaMomPenaD * omg_sym * config_.max_vel_;
                            gradBeta(1,1) += omgstep * penaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                            partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                            cost += omgstep * penaltyWt.moment_weight * violaMomPena;
                            cost_moment += omgstep * penaltyWt.moment_weight * violaMomPena;
                        }
                    }
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                        violaMom = omg_sym * -config_.min_vel_ * dsigma.x() - config_.max_omega_ * dsigma.y() + config_.min_vel_ * config_.max_omega_;
                        if(violaMom > 0){
                            positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = Alpha * (omg_sym * -config_.min_vel_ * ddsigma.x() - config_.max_omega_ * ddsigma.y());
                            gradBeta(1,0) += omgstep * penaltyWt.moment_weight * violaMomPenaD * omg_sym * -config_.min_vel_;
                            gradBeta(1,1) -= omgstep * penaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                            partialGradByTimes(i) += omg * penaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                            cost += omgstep * penaltyWt.moment_weight * violaMomPena;
                            cost_moment += omgstep * penaltyWt.moment_weight * violaMomPena;
                        }
                    }
                }
                // Anti-skid or anti-rollover constraint
                violaCenAcc = dsigma.x()*dsigma.x()*dsigma.y()*dsigma.y() - config_.max_centripetal_acc_*config_.max_centripetal_acc_;
                if(violaCenAcc > 0){
                    positiveSmoothedL1(violaCenAcc, violaCenAccPena, violaCenAccPenaD);
                    gradViolaCAt = 2.0 * Alpha * (dsigma.x() * dsigma.y() * dsigma.y() * ddsigma.x() + dsigma.y() * dsigma.x() * dsigma.x() * ddsigma.y());
                    gradBeta(1,0) += omgstep * penaltyWt.cen_acc_weight * violaCenAccPenaD * (2 * dsigma.x() * dsigma.y() * dsigma.y());
                    gradBeta(1,1) += omgstep * penaltyWt.cen_acc_weight * violaCenAccPenaD * (2 * dsigma.x() * dsigma.x() * dsigma.y());
                    partialGradByTimes(i) += omg * penaltyWt.cen_acc_weight * (violaCenAccPenaD * gradViolaCAt * step + violaCenAccPena / sparseResolution_);
                    cost += omgstep * penaltyWt.cen_acc_weight * violaCenAccPena;
                    cost_centripetal_acc += omgstep * penaltyWt.cen_acc_weight * violaCenAccPena;
                }

                // ==========================================
                // 碰撞约束：对车身检测点检查 SDF 值
                // ==========================================
                // 检测点通过旋转矩阵 ego_R 从车体系变换到世界系
                // 每个检测点评估 SDF: SDF_value = sdf(x,y)
                // violaPos = -SDF_value + safeDis > 0 表示碰撞
                // 惩罚: smoothL1(-SDF + safeDis)
                // 碰撞时记录梯度用于后续链式传播
                if(j != 0) CurrentPointXY+=Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);

                Eigen::Matrix2d ego_R;
                ego_R << cosyaw,-sinyaw, sinyaw, cosyaw;

                bool if_coolision = false;
                Eigen::Vector2d all_grad2Pos; all_grad2Pos.setZero();


                for(auto cp2D:check_point){
                    Eigen::Vector2d bpt = CurrentPointXY + ego_R * cp2D;  // 车体检测点 -> 世界坐标
                    double sdf_value = map_->getDistWithGradBilinear(bpt, gradESDF2d, safeDis);  // SDF 双线性插值 + 梯度
                    violaPos = -sdf_value + safeDis;
                    if (violaPos > 0.0){
                        if_coolision = true;
                        positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
                        all_grad2Pos -= omgstep * penaltyWt.collision_weight * violaPosPenaD * gradESDF2d;
                        help_L << -sinyaw, -cosyaw, cosyaw, -sinyaw;
                        gradViolaPt = -Alpha * dsigma.x() * gradESDF2d.transpose() * help_L * cp2D;
                        
                        gradBeta(0, 0) -= omgstep * penaltyWt.collision_weight * violaPosPenaD * gradESDF2d.transpose() * help_L * cp2D;
                        partialGradByTimes(i) += omg * penaltyWt.collision_weight * (violaPosPenaD * gradViolaPt * step + violaPosPena / sparseResolution_);
                        cost += omgstep * penaltyWt.collision_weight * violaPosPena;
                        cost_corrb += omgstep * penaltyWt.collision_weight * violaPosPena;
                    }
                }
                // 碰撞梯度累积：碰撞时的位置梯度被记录到 VecCoeffChainX/Y
                // 在循环结束后通过链式法则传播到多项式系数和时间梯度
                if(if_coolision){
                    VecCoeffChainX.head(i*(SamNumEachPart+1)+j+1).array() += all_grad2Pos.x();
                    VecCoeffChainY.head(i*(SamNumEachPart+1)+j+1).array() += all_grad2Pos.y();
                }

                // 将所有通过 gradBeta 累积的梯度传播到多项式系数
                // 链式法则: ∂cost/∂c = ∂cost/∂sigma*beta0 + ∂cost/∂dsigma*beta1 + ∂cost/∂ddsigma*beta2
                // 其中 sigma=c^T*beta0, dsigma=c^T*beta1, ddsigma=c^T*beta2
                partialGradByCoeffs.block<6,2>(i*6, 0) += beta0 * gradBeta.row(0) + beta1 * gradBeta.row(1) + beta2 * gradBeta.row(2);
            }
            else{
                // ==========================================
                // 奇索引采样点（Simpson 积分中点, 权重 4）
                // ==========================================
                // 只计算 Simpson 积分所需的量（dx, dy 及其梯度），
                // 不评估约束惩罚以降低计算开销
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s3 * s2;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                s1 += halfstep;
                IntegralAlpha = 1.0 / SamNumEachPart * j;
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

        // ==========================================
        // 段时长均衡约束
        // ==========================================
        // 惩罚单段时长偏离平均时长的 lowBound ~ uppBound 倍数范围
        // cost += ω * (T_i - avgT*bound)^2  当 T_i 超出范围
        // 同时惩罚 avgT 的偏移以保持整体一致
        if( pieceTime[i] < unoccupied_averageT * mean_time_lowBound_){
            cost += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            cost_meanT += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            partialGradByTimes.array() += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_)  * (- mean_time_lowBound_ / TrajNum);
            partialGradByTimes(i) += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
        }
        if (pieceTime[i] > unoccupied_averageT * mean_time_uppBound_){
            cost += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            cost_meanT += penaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            partialGradByTimes.array() += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (-mean_time_uppBound_ / TrajNum);
            partialGradByTimes(i) += penaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
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

    // ==========================================
    // 终端位置约束 (ALM 增广拉格朗日)
    // ==========================================
    // FinalIntegralXYError = 积分终点 - 目标终点
    // 增广拉格朗日项: 0.5 * ρ * ||h(x) + λ/ρ||^2
    // 梯度: ∂L/∂h = ρ*(h + λ/ρ) = ρ*h + λ
    // 该梯度通过 VecCoeffChain 累积后传播到各个积分点
    FinalIntegralXYError = VecTrajFinalXY.back() - finStateXYTheta.head(2);
    cost += 0.5 * (EqualRho[0] * pow(FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0], 2) + EqualRho[1] * pow(FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1], 2));
    cost_endp += 0.5 * (EqualRho[0] * pow(FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0], 2) + EqualRho[1] * pow(FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1], 2));
    if(ifprint){
        ROS_INFO("\033[40;33m iter finStateXY:%f  %f  \033[0m", VecTrajFinalXY.back().x(), VecTrajFinalXY.back().y());
        ROS_INFO("\033[40;33m real finStateXY:%f  %f  \033[0m", finStateXYTheta.x(), finStateXYTheta.y());
        ROS_INFO("error: %f", FinalIntegralXYError.norm());
    }
    VecCoeffChainX.array() += EqualRho[0] * (FinalIntegralXYError.x() + EqualLambda[0]/EqualRho[0]);
    VecCoeffChainY.array() += EqualRho[1] * (FinalIntegralXYError.y() + EqualLambda[1]/EqualRho[1]);


    if(ifprint){
        ROS_INFO("cost: %f", cost);
        ROS_INFO("cost corridor: %f", cost_corrb);
        ROS_INFO("cost end p: %f", cost_endp);
        ROS_INFO("cost v: %f", cost_v);
        ROS_INFO("cost a: %f", cost_a);
        ROS_INFO("cost omega: %f", cost_omega);
        ROS_INFO("cost domega: %f", cost_domega);
        ROS_INFO("cost moment: %f", cost_moment);
        ROS_INFO("cost meanT: %f", cost_meanT);
        ROS_INFO("cost centripetal_acc: %f", cost_centripetal_acc);
    } 
 
    // ==========================================
    // 梯度传播：将碰撞和终端约束的梯度传播到系数和时间
    // ==========================================
    // VecCoeffChainX/Y 累积了碰撞约束和 ALM 终端约束对位置 (x,y) 的梯度
    // 乘以 Simpson 积分系数 IntegralChainCoeff 后：
    // - 通过 VecSingleXGradCS 等传播到多项式系数 ∂x/∂c_theta, ∂x/∂c_s
    // - 通过 VecSingleXGradT 等传播到时间段    ∂x/∂T
    // 注意：此部分必须在碰撞和终端约束计算完成之后执行
    for(int i=0; i<TrajNum; i++){
        ///////////////////////////////////////////////////////////////////////////
        Eigen::VectorXd CoeffX = VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        Eigen::VectorXd CoeffY = VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleXGradCS[i] * CoeffX;
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleXGradCTheta[i] * CoeffX;
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleYGradCS[i] * CoeffY;
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleYGradCTheta[i] * CoeffY;
        ///////////////////////////////////////////////////////////////////////////
        partialGradByTimes(i) += (VecSingleXGradT[i].cwiseProduct(CoeffX)).sum();
        partialGradByTimes(i) += (VecSingleYGradT[i].cwiseProduct(CoeffY)).sum();
    }
}

/**
 * positiveSmoothedL1 — 光滑 L1 惩罚函数（仅正半轴）
 * =================================================
 *
 * 对 x > 0 的区域施加惩罚，在 [0, smoothEps] 区间用四次多项式光滑过渡：
 *   x >= smoothEps: f(x) = x - smoothEps/2,       f'(x) = 1
 *   x < smoothEps:   f(x) = (f4c*x + f3c)*x^3,    f'(x) = (d3c*x + d2c)*x^2
 *
 * 其中 f3c = 1/smoothEps^2, f4c = -0.5/smoothEps^3
 *      d2c = 3/smoothEps^2, d3c = -2/smoothEps^3
 *
 * 该函数保证了 f 在 smoothEps 处 C^2 连续，使得 L-BFGS 等拟牛顿
 * 方法有良好的收敛性质。
 *
 * 用途：对约束违反量 (viola > 0) 施加可微惩罚
 */
inline void MSPlanner::positiveSmoothedL1(const double &x, double &f, double &df){
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

/**
 * backwardGradT — 虚拟时间梯度的反向传播（链式法则）
 * ===================================================
 * gradTau(i) = gradT(i) * dT/dtau(i)
 *
 * dT/dtau 的计算：
 *   tau > 0: T = 0.5*tau^2 + tau + 1
 *             dT/dtau = tau + 1
 *   tau < 0: T = 1 / (0.5*tau^2 - tau + 1)
 *             dT/dtau = (1 - tau) / (0.5*tau^2 - tau + 1)^2
 *
 * 用途：L-BFGS 需要的是 ∂f/∂tau（对优化变量 tau 的梯度），
 *       但约束计算产生的是 ∂f/∂T（对真实时间的梯度），
 *       通过 dT/dtau 将两者关联
 */
template <typename EIGENVEC>
inline void MSPlanner::backwardGradT(const Eigen::VectorXd &tau,
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

/**
 * get_the_predicted_state — 通过 Simpson 积分重建 (x,y,theta) 状态
 * ================================================================
 * 从初始位姿出发，沿最终轨迹积分到指定时间 time，
 * 返回预测的 (x, y, theta) 和速度/加速度/角速度/角加速度。
 *
 * 积分步长为 trajPredictResolution_，使用 Simpson 3点 格式。
 * 剩余不足一个完整步长的部分单独处理。
 *
 * ICR 运动学 dx = ds*cosθ ± dθ*ICR*sinθ (符号取决于 ICR)
 */
void MSPlanner::get_the_predicted_state(const double& time, Eigen::Vector3d& XYTheta, Eigen::Vector3d& VAJ, Eigen::Vector3d& OAJ){

    double check_time = time;
    if(time > final_traj_.getTotalDuration()){
        check_time = final_traj_.getTotalDuration();
    }
    
    double x1, x2, x3, y1, y2, y3;

    XYTheta = final_initStateXYTheta_;

    double step = trajPredictResolution_;
    double halfstep = step / 2.0;
    double step1_6 = step / 6.0;

    int sequence_num = floor(check_time / step);
    double left_time = check_time - sequence_num * step;

    Eigen::Vector2d p1, p2, p3, v1, v2, v3, a3, j3;
    p3 = final_traj_.getPos(0.0);
    v3 = final_traj_.getVel(0.0);
    
    for(int i=0; i<sequence_num; ++i){
        p1 = p3; v1 = v3;
        p2 = final_traj_.getPos(i * step + halfstep);
        v2 = final_traj_.getVel(i * step + halfstep);
        p3 = final_traj_.getPos(i * step + step);
        v3 = final_traj_.getVel(i * step + step);
        if(if_standard_diff_){
            XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
            XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
        }
        else{
            x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
            x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
            x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

            y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
            y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
            y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

            XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
            XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
        }

        XYTheta.z() = p3.x();
    }

    step1_6 = left_time/6.0;
    p1 = p3; v1 = v3;
    p2 = final_traj_.getPos(check_time - left_time / 2.0);
    v2 = final_traj_.getVel(check_time - left_time / 2.0);
    p3 = final_traj_.getPos(check_time);
    v3 = final_traj_.getVel(check_time);

    if(if_standard_diff_){
        XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
        XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
    }
    else{
        x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
        x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
        x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

        y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
        y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
        y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

        XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
        XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
    }

    XYTheta.z() = p3.x();


    a3 = final_traj_.getAcc(check_time);
    j3 = final_traj_.getJer(check_time);

    OAJ << v3.x(), a3.x(), j3.x();
    VAJ << v3.y(), a3.y(), j3.y();
    return; 
}

std::vector<Eigen::Vector3d> MSPlanner::get_the_predicted_state_and_path(const double &start_time, const double &time, 
                                                                       const Eigen::Vector3d &start_XYTheta, Eigen::Vector3d &XYTheta, bool &if_forward){
    std::vector<Eigen::Vector3d> path;
    double check_time = time;
    if(time > final_traj_.getTotalDuration()){
        check_time = final_traj_.getTotalDuration();
    }
    
    double x1, x2, x3, y1, y2, y3;

    XYTheta = start_XYTheta;
    path.push_back(start_XYTheta);

    double step = trajPredictResolution_;
    double halfstep = step / 2.0;
    double step1_6 = step / 6.0;

    int sequence_num = floor((check_time - start_time) / step);
    double left_time = check_time - sequence_num * step - start_time;

    Eigen::Vector2d p1, p2, p3, v1, v2, v3, a3, j3;
    p3 = final_traj_.getPos(start_time);
    v3 = final_traj_.getVel(start_time);
    
    for(int i=0; i<sequence_num; ++i){
        p1 = p3; v1 = v3;
        p2 = final_traj_.getPos(start_time + i * step + halfstep);
        v2 = final_traj_.getVel(start_time + i * step + halfstep);
        p3 = final_traj_.getPos(start_time + i * step + step);
        v3 = final_traj_.getVel(start_time + i * step + step);

        if(if_standard_diff_){
            XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
            XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
        }
        else{
            x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
            x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
            x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

            y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
            y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
            y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

            XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
            XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
        }
        XYTheta.z() = p3.x();
        // path.push_back(XYTheta);
    }

    step1_6 = left_time/6.0;
    p1 = p3; v1 = v3;
    p2 = final_traj_.getPos(check_time - left_time / 2.0);
    v2 = final_traj_.getVel(check_time - left_time / 2.0);
    p3 = final_traj_.getPos(check_time);
    v3 = final_traj_.getVel(check_time);
    if(if_standard_diff_){
        XYTheta.x() += step1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
        XYTheta.y() += step1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
    }
    else{
        x1 = v1.y()*cos(p1.x()) + v1.x() * ICR_.z() * sin(p1.x());
        x2 = v2.y()*cos(p2.x()) + v2.x() * ICR_.z() * sin(p2.x());
        x3 = v3.y()*cos(p3.x()) + v3.x() * ICR_.z() * sin(p3.x());

        y1 = v1.y()*sin(p1.x()) - v1.x() * ICR_.z() * cos(p1.x());
        y2 = v2.y()*sin(p2.x()) - v2.x() * ICR_.z() * cos(p2.x());
        y3 = v3.y()*sin(p3.x()) - v3.x() * ICR_.z() * cos(p3.x());

        XYTheta.x() += step1_6 * (x1 + 4.0*x2 + x3);
        XYTheta.y() += step1_6 * (y1 + 4.0*y2 + y3);
    }
    XYTheta.z() = p3.x();
    // path.push_back(XYTheta);

    if_forward = final_traj_.getPos(time).y() - final_traj_.getPos(start_time).y() > 0.0? true:false;
    
    return path;
}

/**
 * costFunctionCallbackPath — 预处理阶段的代价函数回调
 * ====================================================
 * 与 costFunctionCallback 类似，但使用：
 * - attachPenaltyFunctionalPath 而非 attachPenaltyFunctional
 * - PathpenaltyWt 而非 penaltyWt
 * 预处理阶段主要约束路径形状相似性，不包含碰撞和终端约束
 */
double MSPlanner::costFunctionCallbackPath(void *ptr,
                                         const Eigen::VectorXd &x,
                                         Eigen::VectorXd &g){
    if(x.norm()>1e4){
        return inf;
    }
    MSPlanner &obj = *(MSPlanner *)ptr;
    ++obj.iter_num_;
    int offset = 0;
    Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.TrajNum - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data()+offset, 2, obj.TrajNum - 1);
    offset += 2 * (obj.TrajNum - 1);

    double* gradTailS = g.data()+offset;
    obj.finState(1,0) = x[offset];
    ++offset;

    gradP.setZero();
    obj.Innerpoints = P;
    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.TrajNum);
    Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.TrajNum);
    offset += obj.TrajNum;
    obj.VirtualT2RealT(t, obj.pieceTime);
    gradt.setZero();
    double cost;
    obj.Minco.setTConditions(obj.finState);
    obj.Minco.setParameters(obj.Innerpoints,obj.pieceTime);
    obj.Minco.getEnergy(cost);
    obj.Minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
    obj.Minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    obj.attachPenaltyFunctionalPath(cost);
    obj.Minco.propogateArcYawLenghGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                        obj.gradByPoints, obj.gradByTimes, obj.gradByTailStateS);

    *gradTailS = obj.gradByTailStateS.y();

    cost += obj.PathpenaltyWt.time_weight * obj.pieceTime.sum();

    Eigen::VectorXd rhotimes;
    rhotimes.resize(obj.gradByTimes.size());
    obj.gradByTimes += obj.penaltyWt.time_weight * rhotimes.setOnes();
    gradP = obj.gradByPoints;
    backwardGradT(t, obj.gradByTimes, gradt);
    
    return cost;
}

/**
 * attachPenaltyFunctionalPath — 预处理阶段的约束惩罚
 * ===================================================
 *
 * 与 attachPenaltyFunctional 的核心区别：
 * 1. 不包含碰撞约束（SDF 检查）
 * 2. 不包含向心加速度约束
 * 3. 包含路径相似性约束（bigpath_sdf_weight）：
 *    惩罚积分重建的中间路点与初始前端路点 inner_init_positions[i]
 *    之间的平方距离
 *
 * 惩罚项：
 * - 驱动轮扭矩多边形约束
 * - 加速度/角加速度约束
 * - 段时长均衡
 * - 路径相似性（与初始路点的距离）
 */
void MSPlanner::attachPenaltyFunctionalPath(double &cost){
    double ini_x = iniStateXYTheta.x();
    double ini_y = iniStateXYTheta.y();

    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    int SamNumEachPart = 2 * sparseResolution_;
    double IntegralAlpha, omg;

    double unoccupied_averageT;
    unoccupied_averageT = pieceTime.mean();
    
    double violaPos;

    double violaMom;
    double violaMomPena;
    double violaMomPenaD;

    double cost_bp=0, cost_final_p=0, cost_moment=0, cost_meanT=0;

    Eigen::Matrix2d help_L;
    Eigen::Vector2d gradESDF2d;

    Eigen::VectorXd IntegralChainCoeff(SamNumEachPart + 1);
    IntegralChainCoeff.setZero();
    for(int i=0; i<sparseResolution_; i++){
        IntegralChainCoeff.block(2*i,0,3,1) += Eigen::Vector3d(1.0, 4.0, 1.0);
    }

    std::vector<Eigen::VectorXd> VecIntegralX(TrajNum);
    std::vector<Eigen::VectorXd> VecIntegralY(TrajNum);
    std::vector<Eigen::Vector2d> VecTrajFinalXY(TrajNum+1);
    VecTrajFinalXY[0] = Eigen::Vector2d(ini_x, ini_y);

    std::vector<Eigen::MatrixXd> VecSingleXGradCS(TrajNum);
    std::vector<Eigen::MatrixXd> VecSingleXGradCTheta(TrajNum);
    std::vector<Eigen::VectorXd> VecSingleXGradT(TrajNum);
    std::vector<Eigen::MatrixXd> VecSingleYGradCS(TrajNum);
    std::vector<Eigen::MatrixXd> VecSingleYGradCTheta(TrajNum);
    std::vector<Eigen::VectorXd> VecSingleYGradT(TrajNum);

    Eigen::VectorXd VecCoeffChainX(TrajNum*(SamNumEachPart+1));VecCoeffChainX.setZero();
    Eigen::VectorXd VecCoeffChainY(TrajNum*(SamNumEachPart+1));VecCoeffChainY.setZero();
    // Eigen::Vector2d CurrentPointXY(ini_x, ini_y);

    for(int i=0; i<TrajNum; i++){
        const Eigen::Matrix<double, 6, 2> &c = Minco.getCoeffs().block<6,2>(6*i, 0);
        double step = pieceTime[i] / sparseResolution_;
        double halfstep = step / 2;
        double CoeffIntegral = pieceTime[i] / sparseResolution_ / 6;
        Eigen::MatrixXd SingleXGradCS(6,SamNumEachPart+1);
        Eigen::MatrixXd SingleXGradCTheta(6,SamNumEachPart+1);
        Eigen::VectorXd SingleXGradT(SamNumEachPart+1);
        Eigen::MatrixXd SingleYGradCS(6,SamNumEachPart+1);
        Eigen::MatrixXd SingleYGradCTheta(6,SamNumEachPart+1);
        Eigen::VectorXd SingleYGradT(SamNumEachPart+1);

        Eigen::VectorXd IntegralX(sparseResolution_);IntegralX.setZero();
        Eigen::VectorXd IntegralY(sparseResolution_);IntegralY.setZero();
        s1 = 0.0;
        for(int j=0; j<=SamNumEachPart; j++){
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
                IntegralAlpha = 1.0 / SamNumEachPart * j;
                omg = (j==0||j==SamNumEachPart)? 0.5:1;
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
                    if(j!=SamNumEachPart){
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
                    if(j!=SamNumEachPart){
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
                        positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                        gradViolaMt = Alpha * (omg_sym * config_.max_vel_ * ddsigma.x() + config_.max_omega_ * ddsigma.y());
                        gradBeta(1,0) += omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * omg_sym * config_.max_vel_;
                        gradBeta(1,1) += omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                        partialGradByTimes(i) += omg * PathpenaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                        cost += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                        cost_moment += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                    }
                }
                for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2){
                    violaMom = omg_sym * -config_.min_vel_ * dsigma.x() - config_.max_omega_ * dsigma.y() + config_.min_vel_ * config_.max_omega_;
                    if(violaMom > 0){
                        positiveSmoothedL1(violaMom, violaMomPena, violaMomPenaD);
                        gradViolaMt = Alpha * (omg_sym * -config_.min_vel_ * ddsigma.x() - config_.max_omega_ * ddsigma.y());
                        gradBeta(1,0) += omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * omg_sym * -config_.min_vel_;
                        gradBeta(1,1) -= omg * step * PathpenaltyWt.moment_weight * violaMomPenaD * config_.max_omega_;
                        partialGradByTimes(i) += omg * PathpenaltyWt.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / sparseResolution_);
                        cost += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                        cost_moment += omg * step * PathpenaltyWt.moment_weight * violaMomPena;
                    }
                }

                double violaAcc = ddsigma.y()*ddsigma.y() - config_.max_acc_*config_.max_acc_;
                double violaAlp = ddsigma.x()*ddsigma.x() - config_.max_domega_*config_.max_domega_;
                double violaAccPena, violaAccPenaD, violaAlpPena, violaAlpPenaD;
                if(violaAcc > 0){
                    positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
                    double gradViolaAT = 2.0 * Alpha * ddsigma.y() * dddsigma.y();
                    gradBeta(2,1) +=  omg * step * PathpenaltyWt.acc_weight * violaAccPenaD * 2.0 * ddsigma.y();
                    partialGradByTimes(i) += omg * PathpenaltyWt.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / sparseResolution_);
                    cost += omg * step * PathpenaltyWt.acc_weight * violaAccPena;
                    cost_moment += omg * step * PathpenaltyWt.acc_weight * violaAccPena;
                }
                if(violaAlp > 0){
                    positiveSmoothedL1(violaAlp, violaAlpPena, violaAlpPenaD);
                    double gradViolaDOT = 2.0 * Alpha * ddsigma.x() * dddsigma.x();
                    gradBeta(2,0) += omg * step * PathpenaltyWt.domega_weight * violaAlpPenaD * 2.0 * ddsigma.x();
                    partialGradByTimes(i) += omg * PathpenaltyWt.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / sparseResolution_);
                    cost += omg * step * PathpenaltyWt.domega_weight * violaAlpPena;
                    cost_moment += omg * step * PathpenaltyWt.domega_weight * violaAlpPena;
                }

                partialGradByCoeffs.block<6,2>(i*6, 0) += beta0 * gradBeta.row(0) + beta1 * gradBeta.row(1) + beta2 * gradBeta.row(2);
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
                IntegralAlpha = 1.0 / SamNumEachPart * j;
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

        if( pieceTime[i] < unoccupied_averageT * mean_time_lowBound_){
            cost += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            cost_meanT += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
            partialGradByTimes.array() += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_)  * (- mean_time_lowBound_ / TrajNum);
            partialGradByTimes(i) += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_lowBound_);
        }
        if (pieceTime[i] > unoccupied_averageT * mean_time_uppBound_){
            cost += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            cost_meanT += PathpenaltyWt.mean_time_weight * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_) * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
            partialGradByTimes.array() += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_)  * (- mean_time_uppBound_ / TrajNum);
            partialGradByTimes(i) += PathpenaltyWt.mean_time_weight * 2.0 * (pieceTime[i] - unoccupied_averageT * mean_time_uppBound_);
        }

        // ==========================================
        // 路径相似性约束：惩罚积分重建位置与初始路点的偏差
        // ==========================================
        // violaPos = ||innerpointXY - inner_init_positions[i]||^2
        // 梯度传播到所有之前的积分点
        Eigen::Vector2d innerpointXY = VecTrajFinalXY[i+1];
        violaPos = (innerpointXY - inner_init_positions[i].head(2)).squaredNorm();
        VecCoeffChainX.head((i+1)*(SamNumEachPart+1)).array() += PathpenaltyWt.bigpath_sdf_weight * 2.0 * (innerpointXY.x() - inner_init_positions[i].x());
        VecCoeffChainY.head((i+1)*(SamNumEachPart+1)).array() += PathpenaltyWt.bigpath_sdf_weight * 2.0 * (innerpointXY.y() - inner_init_positions[i].y());
        cost += PathpenaltyWt.bigpath_sdf_weight * violaPos;
        cost_bp += PathpenaltyWt.bigpath_sdf_weight * violaPos;

    }

    if(ifprint){
        ROS_INFO("cost: %f", cost);
        ROS_INFO("cost big path dis: %f", cost_bp);
        ROS_INFO("cost final p: %f", cost_final_p);
        ROS_INFO("cost moment: %f", cost_moment);
    } 

    for(int i=0; i<TrajNum; i++){
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleXGradCS[i] * VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleXGradCTheta[i] * VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByCoeffs.block<6,1>(i*6, 1) += VecSingleYGradCS[i] * VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByCoeffs.block<6,1>(i*6, 0) += VecSingleYGradCTheta[i] * VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff);
        partialGradByTimes(i) += (VecSingleXGradT[i].cwiseProduct(VecCoeffChainX.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff))).sum();
        partialGradByTimes(i) += (VecSingleYGradT[i].cwiseProduct(VecCoeffChainY.block(i*(SamNumEachPart+1),0,SamNumEachPart+1,1).cwiseProduct(IntegralChainCoeff))).sum();
    }
}

/**
 * mincoPathPub — 将 MINCO 轨迹通过 Simpson 积分重建为世界坐标路径并发布
 * =========================================================================
 * 用于 ROS 可视化，采样密度为 sparseResolution_ * 10（比优化时高 10 倍）
 * 每条发布的 nav_msgs::Path 包含所有积分点的位姿
 */
void MSPlanner::mincoPathPub(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta, const ros::Publisher &publisher){

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
                // outFile << currPos.x() << " " << currPos.y() << std::endl;
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
                // outFile << currPos.x() << " " << currPos.y() << std::endl;
            }
        }
        VecIntegralX[i] = IntegralX;
        VecIntegralY[i] = IntegralY;
        VecYaw[i] = Yaw;
        sumT += pieceTime[i];
    }

    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    Eigen::Vector2d pos(ini_x, ini_y);
    for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
            pos.x() += VecIntegralX[i][j];
            pos.y() += VecIntegralY[i][j];
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = pos.x();
            pose.pose.position.y = pos.y();
            pose.pose.position.z = 0.15;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(VecYaw[i][j]);
            path.poses.push_back(pose);
            // outFile2 << pos.x() << " " << pos.y() << std::endl;
        }
    }
    publisher.publish(path);
    // outFile.close();
    // outFile2.close();
}

/**
 * mincoPointPub — 将轨迹重建为 SPHERE_LIST 标记（只发布段终点）
 * ================================================================
 * 每个段的终点被发布为一个球体标记，颜色由参数指定
 */
void MSPlanner::mincoPointPub(const Trajectory<5,2> &final_traj, const Eigen::Vector3d &start_state_XYTheta, const ros::Publisher &publisher, const Eigen::Vector3d &color){
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
        // VecTrajFinalXY[i+1] = Eigen::Vector2d(IntegralX[IntegralX.size()-1], IntegralY[IntegralX.size()-1]);
        sumT += pieceTime[i];
    }

    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.ns = "Innerpoint";
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    publisher.publish(marker);
    
    
    marker.action = visualization_msgs::Marker::ADD;    
    marker.id = 0;
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;
    marker.color.a = 1.0;
    marker.color.r = color.x()/255.0;
    marker.color.g = color.y()/255.0;
    marker.color.b = color.z()/255.0;
    Eigen::Vector2d pos(ini_x, ini_y);
    for(u_int i=0; i<VecIntegralX.size(); i++){
        for(u_int j=0; j<VecIntegralX[i].size(); j++){
            pos.x() += VecIntegralX[i][j];
            pos.y() += VecIntegralY[i][j];
            geometry_msgs::Point pt;
            pt.x = pos.x();
            pt.y = pos.y();
            pt.z = 0.15;
            if(j == VecIntegralX[i].size()-1){
                marker.points.push_back(pt);
            }
        }
    }
    publisher.publish(marker);
}


inline double MSPlanner::normlize_angle(double angle){
    if(angle > M_PI) angle -= 2 * M_PI;
    else if(angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void MSPlanner::Collision_point_Pub(){
    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::PointXYZRGB  pt;
    for(u_int i=0;i<collision_point_.size();i++){
        pt.x = collision_point_[i][0];
        pt.y = collision_point_[i][1];
        pt.z = 0.3;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        colored_pcl_ptr->points.push_back(pt); 
    }
    cloudMap.height = colored_pcl_ptr->points.size();
    cloudMap.width = 1;
    // cloudMap.is_dense = true;
    pcl::toROSMsg(*colored_pcl_ptr,globalMap_pcd);
    globalMap_pcd.header.stamp = ros::Time::now();
    globalMap_pcd.header.frame_id = "world";
    CollisionpointPub.publish(globalMap_pcd);
}

void MSPlanner::pub_inner_init_positions(const std::vector<Eigen::Vector3d> &inner_init_positions){
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.ns = "Innerpoint";
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    innerinitpositionsPoint.publish(marker);
    
    
    marker.action = visualization_msgs::Marker::ADD;    
    marker.id = 0;
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;
    marker.color.a = 1.0;
    marker.color.r = 193.0/255.0;
    marker.color.g = 125.0/255.0;
    marker.color.b = 17.0/255.0;
    for(u_int i=0; i<inner_init_positions.size(); i++){
        geometry_msgs::Point pt;
        pt.x = inner_init_positions[i].x();
        pt.y = inner_init_positions[i].y();
        pt.z = 0.15;
        marker.points.push_back(pt);
    }
    innerinitpositionsPoint.publish(marker);
}