#include "plan_env/sdf_map.h"

// ============================================================================
// pointCloudCallback: 激光点云回调函数
// 功能: 接收激光雷达点云,通过TF变换将点云从传感器坐标系转换到世界坐标系,
//       并提取机器人当前位置和姿态,设置occupancy地图需要更新的标志位。
// 流程:
//   1. 通过TF2查询激光雷达坐标系到世界坐标系的变换关系
//   2. 提取机器人在世界坐标系下的2D位置(x,y)和偏航角(yaw)
//   3. 将整个点云从传感器坐标系变换到世界坐标系
//   4. 将变换后的点云转换为PCL格式并存储
//   5. 设置occ_need_update_标志为true,触发下一次地图更新
// ============================================================================
void SDFmap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  static tf2_ros::Buffer tf_buffer(ros::Duration(10.0));
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  // Get transformation information
  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = tf_buffer.lookupTransform("world", msg->header.frame_id,
                                                  msg->header.stamp, ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
  }

  odom_pos_.head(2) = Eigen::Vector2d(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
  odom_pos_[2] = tf::getYaw(transformStamped.transform.rotation);

  // Perform coordinate transformation
  sensor_msgs::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transformStamped);

  // Convert to PCL format
  pcl::fromROSMsg(transformed_cloud, cloud_);
  
  occ_need_update_ = true;
}

// ============================================================================
// updateOccupancyCallback: 定时器触发的occupancy地图更新回调
// 功能: 根据当前机器人位置,定义以机器人为中心的局部检测窗口,
//       在该窗口内执行光线投射(raycasting)更新occupancy grid。
// 两种模式:
//   if_perspective_==false: 基于射线投射的模式
//     1. 根据机器人位置和detection_range_计算局部窗口的x/y上下界
//     2. 调用raycastProcess()执行从机器人到每个激光点的Bresenham射线遍历
//     3. 可选调用cirSupRaycastProcess()补充圆形射线投射,填补激光盲区
//     4. RemoveOutliers()清除被无障碍网格包围的孤立未知网格
//     5. 根据occupancy_map_的对数几率更新gridmap_状态(仅更新原本为Unknown的网格)
//   if_perspective_==true: 简单的投影模式
//     1. 将局部窗口内所有Unknown网格设为Unoccupied
//     2. 直接将点云中的每个点所在的网格设为Occupied
// 最后设置has_map_=true,触发ESDF更新标志esdf_need_update_
// ============================================================================
void SDFmap::updateOccupancyCallback(const ros::TimerEvent& /*event*/){
  if(!occ_need_update_) return;

  if(!if_perspective_){
    ros::Time t1, t2;
    t1 = ros::Time::now();

    // 计算以机器人为中心的局部检测窗口边界
    // x_lower_/x_upper_: 以detection_range_为半径,x方向窗口范围,不超出全局地图边界
    // y_lower_/y_upper_: y方向同理
    x_lower_ = std::max(odom_pos_.x() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_pos_.x() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_pos_.y() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_pos_.y() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_upper_);

    X_SIZE_ = ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    raycastProcess();

    if(if_cirSupRaycast_){
      static int cirSup = 1;
      cirSup ++;
      if(cirSup%3==0){   // 每3帧执行一次圆形补充射线投射,降低计算量
        cirSupRaycastProcess();
        cirSup = 1;
      }
    }

    RemoveOutliers();
    
    // 根据occupancy_map_的对数几率(log-odds)更新gridmap_状态
    // 重要设计: 只更新原本Unknown的网格为Unoccupied或Occupied,
    //          不会将已标记为Occupied的网格降级为Unoccupied（避免消除已有障碍物）
    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    // // 旧方案: 会将障碍物也当作自由空间处理
    // for (int x=min_id.x(); x<=max_id.x(); x++) {
    //   for (int y=min_id.y(); y<=max_id.y(); y++){
    //     if(occupancy_map_[Index2Vectornum(x,y)] >= clamp_min_log_ && occupancy_map_[Index2Vectornum(x,y)] <= min_occupancy_log_){
    //       gridmap_[Index2Vectornum(x,y)] = Unoccupied;
    //     }
    //     else if(occupancy_map_[Index2Vectornum(x,y)] > min_occupancy_log_){
    //       gridmap_[Index2Vectornum(x,y)] = Occupied;
    //     }
    //   }
    // }
    // 当前方案: 不会将已标记为Occupied的网格改回Unoccupied!!
    for (int x=min_id.x(); x<=max_id.x(); x++) {
      for (int y=min_id.y(); y<=max_id.y(); y++){
        int vecIndex = Index2Vectornum(x,y);
        if(gridmap_[vecIndex] == Unknown && occupancy_map_[vecIndex] >= clamp_min_log_ && occupancy_map_[vecIndex] <= min_occupancy_log_){
          gridmap_[vecIndex] = Unoccupied;
        }
        else if(occupancy_map_[vecIndex] > min_occupancy_log_){
          gridmap_[vecIndex] = Occupied;
        }
      }
    }
  }

  
  else{
    // perspective模式: 简单投影
    // 局部窗口内所有Unknown网格 -> Unoccupied
    // 点云点所在网格 -> Occupied
    x_lower_ = std::max(odom_pos_.x() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_pos_.x() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_pos_.y() - ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_pos_.y() + ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_upper_);

    X_SIZE_ = ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    for (int x=min_id.x(); x<=max_id.x(); x++) {
      for (int y=min_id.y(); y<=max_id.y(); y++){
        if(gridmap_[Index2Vectornum(x,y)] == Unknown)
          gridmap_[Index2Vectornum(x,y)] = Unoccupied;
      }
    }

    for(auto point:cloud_.points){
      Eigen::Vector2d coord = Eigen::Vector2d(point.x, point.y);
      if(!isInGloMap(coord)){
        continue;
      }
      Eigen::Vector2i idx = coord2gridIndex(coord);
      gridmap_[Index2Vectornum(idx)] = Occupied;
    }
  }


  has_map_ = true;
  esdf_need_update_ = true;
  occ_need_update_ = false;
}

// ============================================================================
// raycastProcess: Bresenham射线投射处理
// 功能: 遍历点云中的每个激光点,从机器人位置出发沿射线方向进行栅格遍历,
//       标记射线路径上的栅格为"未击中"(free),终点栅格为"击中"(occupied)。
// 算法流程:
//   1. 对每个激光点cur_point:
//      a) 如果点超出全局地图范围: 调用closetPointInMap找到地图边界内的最近点,
//         并限制在detection_range_内,标记为未击中(occ=0)
//      b) 如果点距离超过detection_range_: 截断到检测范围边界,标记为未击中
//      c) 如果点在范围内: 标记为击中(occ=1)
//   2. 使用Bresenham算法(getGridsBetweenPoints2D)获取从机器人到cur_point
//      的所有中间栅格,将这些中间栅格全部标记为未击中(occ=0)
//   3. 最后调用updateOccupancyMap()将对数几率更新应用到occupancy_map_
// ============================================================================
void SDFmap::raycastProcess(){
  int points_cnt = cloud_.points.size();
  
  update_odom_ = odom_pos_.head(2);
  Eigen::Vector2d cur_point;
  int vox_idx;
  double length;

  Eigen::Vector3d ray_pt;
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);

  RayCaster raycaster;

  for (int i = 0; i < points_cnt; ++i) {
    cur_point << cloud_.points[i].x, cloud_.points[i].y;
    if(!isInGloMap(cur_point)){
      cur_point = closetPointInMap(cur_point, update_odom_);
      length = (cur_point - update_odom_).norm();
      if(length > detection_range_){
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
      }
      vox_idx = setCacheOccupancy(cur_point, 0);
    }
    else {
      length = (cur_point - update_odom_).norm();
      if (length > detection_range_) {
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
        vox_idx = setCacheOccupancy(cur_point, 0);
      } else {
        vox_idx = setCacheOccupancy(cur_point, 1);
      }
    }
    std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(coord2gridIndex(update_odom_), coord2gridIndex(cur_point));
    
    int size = line.size() - 1;

    for (int i=0; i<size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }

  }

  updateOccupancyMap();
}

// ============================================================================
// cirSupRaycastProcess: 圆形补充射线投射
// 功能: 当激光雷达存在盲区(如视野受限)时,通过从机器人向外生成圆形分布的
//       采样点并执行射线投射,保证机器人周围有足够的安全自由空间。
// 工作原理:
//   1. 生成围绕机器人的圆形采样点集合:
//      - 在水平和垂直方向分别扫描,在检测范围内以grid_interval_为步长采样
//      - 采样点覆盖检测范围的整个矩形区域的外边界
//   2. 如果启用了水平视野限制(hrz_limited_),过滤超出激光水平视场角的采样点
//   3. 对每个采样点:
//      a) 使用RayCaster从机器人到采样点执行射线投射
//      b) 如果在射线路径上遇到已标记为Occupied的网格(或其邻居),停止并跳过
//      c) 否则将路径上的中间网格标记为未击中(occ=0)
//   4. 直接从cache_voxel_队列中取出缓存的体素,执行对数几率更新
// 注: 此函数每3帧调用一次(由updateOccupancyCallback中的cirSup计数器控制),
//      以平衡计算负载和安全性。
// ============================================================================
void SDFmap::cirSupRaycastProcess(){
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
  double length;
  int vox_idx;

  std::vector<Eigen::Vector2d> cir_points;
  // 生成圆形采样点: 在检测范围的水平边框上采样
  for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_+1e-10;  x += 2*detection_range_){
    for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_+1e-10;  y += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }
  // 生成圆形采样点: 在检测范围的垂直边框上采样
  for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_+1e-10;  y += 2*detection_range_){
    for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_+1e-10;  x += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }

  RayCaster raycaster;

  for(auto cir_point:cir_points){
   
    if(hrz_limited_){
      double angle = atan2(cir_point.y() - odom_pos_.y(), cir_point.x() - odom_pos_.x());
      angle = normalize_angle(angle - odom_pos_.z());
      // 过滤超出激光水平视场角的采样点
      if(angle < -hrz_laser_range_dgr_/2.2 || angle >hrz_laser_range_dgr_/2.2){
        continue;
      }
    }

    if(!isInGloMap(cir_point)){
      cir_point = closetPointInMap(cir_point, update_odom_);
    }

    length = (cir_point - update_odom_).norm();
    if(length > detection_range_){
      cir_point = (cir_point - update_odom_) / length * detection_range_ + update_odom_;
    }

    std::vector<Eigen::Vector2i> line;
    // 使用RayCaster从机器人位置到采样点进行3D射线投射
    raycaster.setInput(Eigen::Vector3d(cir_point.x(), cir_point.y(), 0.1) / grid_interval_, Eigen::Vector3d(update_odom_.x(), update_odom_.y(), 0.1) / grid_interval_);

    bool occ = false;
    Eigen::Vector3d ray_pt;
    while (raycaster.step(ray_pt)) {
      Eigen::Vector2d tmp = (ray_pt.head(2) + half) * grid_interval_;
      Eigen::Vector2i tmp_idx = coord2gridIndex(tmp);
      line.emplace_back(tmp_idx);
      // 检查射线路径上是否遇到障碍物(或其相邻网格),如果遇到则跳过此采样点
      if(gridmap_[Index2Vectornum(coord2gridIndex(tmp))] == Occupied ||
         (tmp_idx.y() < GLY_SIZE_ && gridmap_[Index2Vectornum(coord2gridIndex(tmp))+1] == Occupied) ||
         (tmp_idx.y() > 0 && gridmap_[Index2Vectornum(coord2gridIndex(tmp))-1] == Occupied) ||
         (tmp_idx.x() < GLX_SIZE_ && gridmap_[Index2Vectornum(coord2gridIndex(tmp))+GLY_SIZE_] == Occupied) ||
         (tmp_idx.x() > 0 && gridmap_[Index2Vectornum(coord2gridIndex(tmp))-GLY_SIZE_] == Occupied)){
        occ = true;
        break;
      }
    }
    if(occ) continue;

    int size = line.size()-1;
    // 将射线路径上的中间网格标记为未击中(自由空间)
    for (int i=0; i<size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }
  }

  // updateOccupancyMap();

  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 4*count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    log_odds_update = 0.0;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
  }

}

// ============================================================================
// updateOccupancyMap: 将对数几率(log-odds)更新应用到occupancy_map_
// 功能: 从cache_voxel_队列中取出所有被缓存的体素索引,根据每个体素的
//       击中/未击中计数计算对数几率增量,更新occupancy_map_。
// 对数几率更新规则:
//   - 如果count_hit_ >= count_hit_and_miss_ - 3*count_hit_: 使用prob_hit_log_(正更新)
//   - 否则: 使用prob_miss_log_(负更新)
//   - 更新值被钳制在[clamp_min_log_, clamp_max_log_]范围内
//   - 如果体素不在局部窗口内,直接设为clamp_min_log_(视为自由)
// 计数清零: 每次更新后,将count_hit_和count_hit_and_miss_清零,准备下一帧
// ============================================================================
void SDFmap::updateOccupancyMap(){
//   - 更新值被钳制在[clamp_min_log_, clamp_max_log_]范围内
//   - 如果体素不在局部窗口内,直接设为clamp_min_log_(视为自由)
// 计数清零: 每次更新后,将count_hit_和count_hit_and_miss_清零,准备下一帧
// ============================================================================
  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    // 对数几率更新决策:
    // 命中数 >= 总数 - 3*命中数  => 视为击中,使用正log更新
    // 命中数 <  总数 - 3*命中数  => 视为未击中,使用负log更新
    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 3*count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    // 处理完当前体素后,清零计数器
    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    // 已经达到正/负饱和值的体素跳过更新
    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    // 如果体素不在当前局部窗口内,强制设为最小log值(自由)
    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    // 应用对数几率更新,并钳制在合法范围
    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
  }
}

// ============================================================================
// RemoveOutliers: 移除异常孤立Unknown网格
// 功能: 遍历检测范围内的所有网格,将那些被四周Unoccupied网格包围的
//       Unknown网格也设为Unoccupied,并确保机器人周围3x3区域没有Unknown。
// 流程:
//   1. 在检测范围内以grid_interval_为步长生成密集采样点
//   2. 对于地图内部的每个采样点:
//      如果该网格是Unknown且其上下左右4个邻居都是Unoccupied,
//      则该网格也被设为Unoccupied(可能是传感器噪声引起的孤立未知点)
//   3. 将机器人位置周围3x3区域内的所有Unknown网格强制设为Unoccupied,
//      保证机器人当前位置所在区域是已知安全的
// ============================================================================
void SDFmap::RemoveOutliers(){
  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_+1e-10;  x += grid_interval_){
    for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_+1e-10;  y += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }

  // 忽略地图边界(最外层网格),避免越界访问
  double xlow = global_x_lower_ + grid_interval_;
  double xup = global_x_upper_ - grid_interval_;
  double ylow = global_y_lower_ + grid_interval_;
  double yup = global_y_upper_ - grid_interval_;
  for(auto cir_point:cir_points){
    if(cir_point.x() > xlow && cir_point.x() < xup && cir_point.y() > ylow && cir_point.y() < yup){
      if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] == Unknown){
        // 检查上下左右4邻居是否都是Unoccupied
        if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))+1] == Unoccupied &&
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point))-1] == Unoccupied &&
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point))+GLY_SIZE_] == Unoccupied &&
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point))-GLY_SIZE_] == Unoccupied){
          // 4邻居都是自由空间,则该Unknown网格可能是噪声,设为自由
          gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] = Unoccupied;
        }
      }
    }

  }
  // 保证机器人周围3x3区域没有Unknown,确保局部安全
  Eigen::Vector2i idx = coord2gridIndex(Eigen::Vector2d(odom_pos_.x(), odom_pos_.y()));
  for(int i=-1; i<=1; i++){
    for(int j=-1; j<=1; j++){
      if(gridmap_[Index2Vectornum(idx.x()+i, idx.y()+j)] == Unknown){
        gridmap_[Index2Vectornum(idx.x()+i, idx.y()+j)] = Unoccupied;
      }
    }
  }

}

// ============================================================================
// setCacheOccupancy (坐标版本): 缓存击中/未击中计数,用于后续对数几率更新
// 功能: 将给定坐标位置的体素标记为击中(occ=1)或未击中(occ=0),
//       累计count_hit_and_miss_(总射线数)和count_hit_(击中数)。
// 参数:
//   pos: 世界坐标系下的2D位置
//   occ: 1表示击中(occupied), 0表示未击中(free)
// 返回值:
//   体素的一维索引(idx_ctns),失败返回-1
// 实现细节:
//   首次命中某个体素时,将其加入cache_voxel_队列,供updateOccupancyMap批量处理
// ============================================================================
int SDFmap::setCacheOccupancy(Eigen::Vector2d pos, int occ) {
  if (occ != 1 && occ != 0) return -1;

  Eigen::Vector2i idx = coord2gridIndex(pos);
  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}


// ============================================================================
// setCacheOccupancy (索引版本): 缓存击中/未击中计数(通过grid索引)
// 功能: 与坐标版本相同,但直接接收栅格索引,避免重复坐标转换
// 参数:
//   idx: 栅格坐标索引(Eigen::Vector2i)
//   occ: 1表示击中(occupied), 0表示未击中(free)
// ============================================================================
int SDFmap::setCacheOccupancy(Eigen::Vector2i idx, int occ) {
  if (occ != 1 && occ != 0) return -1;

  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}


// ============================================================================
// getGridsBetweenPoints2D: Bresenham直线栅格遍历算法
// 功能: 实现2D Bresenham直线算法,返回从起点到终点之间的所有栅格坐标。
//       用于raycastProcess中从机器人位置到激光点之间的射线栅格遍历。
// 参数:
//   start: 起始栅格坐标
//   end:   终点栅格坐标
// 返回值:
//   包含从start到end(含端点)的所有中间栅格坐标的vector
// 算法: 标准Bresenham画线算法,使用误差累积决定何时在x/y方向步进。
//       dx, dy: 起点到终点的绝对差值
//       sx, sy: 步进方向(+1或-1)
//       err: 决策变量,决定下一步是沿x还是y移动
// ============================================================================
std::vector<Eigen::Vector2i> SDFmap::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
    std::vector<Eigen::Vector2i> line;

    int dx = abs(end.x() - start.x());
    int dy = abs(end.y() - start.y());
    int sx = (start.x() < end.x()) ? 1 : -1;
    int sy = (start.y() < end.y()) ? 1 : -1;
    int err = dx - dy;  // Bresenham决策变量

    double x0 = start.x();
    double y0 = start.y();

    while (true) {
        line.emplace_back(x0, y0);
        if (x0 == end.x() && y0 == end.y()) break;
        int e2 = 2 * err;
        if (e2 > -dy) {    // 沿x方向步进
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {     // 沿y方向步进
            err += dx;
            y0 += sy;
        }
    }

    return line;
}

void SDFmap::updateESDFCallback(const ros::TimerEvent& /*event*/){
  if(!esdf_need_update_) return;
  updateESDF2d();

  has_esdf_ = true;
  esdf_need_update_ = false;
  
}

void SDFmap::visCallback(const ros::TimerEvent& /*event*/){
  if(has_map_){
    publish_gridmap();
    // exit(0);
  }
  if(has_esdf_){
    publish_ESDF();
    // publish_ESDFGrad();
  }
}

inline void SDFmap::grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size){
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0);
  Eigen::Vector3d z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;

  float insert_interval = 0.5;
  for(float i=-size.x()/2;i<=size.x()/2;i+=grid_interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=grid_interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=grid_interval_*insert_interval){
        Eigen::Vector3d point = location+i*x+j*y+k*z;
        setObs(point);
      }
}

// ============================================================================
// gridIndex2coordd: 栅格坐标转世界坐标
// 功能: 将栅格索引(index)转换为世界坐标系下的实际坐标。
//       栅格中心 = (index + 0.5) * grid_interval_ + global_origin
//       注意加上0.5是为了得到栅格单元的中心点坐标。
// ============================================================================
Eigen::Vector2d SDFmap::gridIndex2coordd(const Eigen::Vector2i &index){
  Eigen::Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)index(1) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const int &x, const int &y){
  Eigen::Vector2d pt;
  pt(0) = ((double)x + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)y + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

// ============================================================================
// coord2gridIndex: 世界坐标转栅格坐标
// 功能: 将世界坐标系下的2D坐标转换为栅格索引。
//       公式: idx = floor((pt - global_origin) * inv_grid_interval_)
//       结果被钳制在[0, GLX_SIZE_-1] x [0, GLY_SIZE_-1]范围内,防止越界。
// ============================================================================
Eigen::Vector2i SDFmap::coord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_), 0), GLX_SIZE_ - 1),
      std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_), 0), GLY_SIZE_ - 1);
  return idx;
}

void SDFmap::setObs(const Eigen::Vector3d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ ||
      coord_x >= global_x_upper_ || coord_y >= global_y_upper_ )
    return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE_ + idx_y] = Occupied;
}

void SDFmap::setObs(const Eigen::Vector2d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ ||
      coord_x >= global_x_upper_ || coord_y >= global_y_upper_ )
    return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE_ + idx_y] = Occupied;
}

// ============================================================================
// vectornum2gridIndex: 一维索引转栅格坐标
// 功能: 将扁平化的一维数组索引num转换为2D栅格坐标(x,y)。
//       转换公式: x = num / GLY_SIZE_, y = num % GLY_SIZE_
//       存储顺序为行主序(row-major): x * GLY_SIZE_ + y
// ============================================================================
Eigen::Vector2i SDFmap::vectornum2gridIndex(const int &num){
  Eigen::Vector2i index;
  index(0) = num / GLY_SIZE_;
  index(1) = num % GLY_SIZE_;
  return index;
}

// ============================================================================
// Index2Vectornum: 栅格坐标转一维索引
// 功能: 将2D栅格坐标(x,y)转换为扁平化的一维数组索引。
//       存储顺序为行主序(row-major): index = x * GLY_SIZE_ + y
//       有两个重载: 一个接收分开的x,y整数,一个接收Eigen::Vector2i
// ============================================================================
int SDFmap::Index2Vectornum(const int &x, const int &y){
  return x * GLY_SIZE_ + y;
}

int SDFmap::Index2Vectornum(const Eigen::Vector2i &index){
  return index.x() * GLY_SIZE_ + index.y();
}

// ============================================================================
// publish_gridmap: 可视化发布occupancy grid地图
// 功能: 将gridmap_转换为PointCloud2消息发布,用于RViz可视化。
//       障碍物(Occupied)用intensity=0.0(深色)显示,
//       未知区域(Unknown)用intensity=8.0(亮色)显示,
//       自由空间(Unoccupied)不显示(已被注释掉)。
//       额外添加一个标记点(100,100)作为参考点。
// ============================================================================
void SDFmap::publish_gridmap(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;
  for(int idx = 1;idx < GLXY_SIZE_;idx++){
    // if(gridmap_[idx]==Unoccupied){
    //   Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
    //   pcl::PointXYZI pt;
    //   pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
    //   pt.intensity = 8.0;
    //   cloud_vis.points.push_back(pt);
    // }
    if(gridmap_[idx]==Occupied){
      Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
      pt.intensity = 0.0;
      cloud_vis.points.push_back(pt);
    }
    if(gridmap_[idx]==Unknown){
      Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
      pt.intensity = 8.0;
      cloud_vis.points.push_back(pt);
    }
  }

  pcl::PointXYZI pt;
  pt.x = 100.0; pt.y = 100.0; pt.z = 0.1;
  pt.intensity = 10.0;
  cloud_vis.points.push_back(pt);

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "world";
  pub_gridmap_.publish(map_vis);
}

uint8_t SDFmap::CheckCollisionBycoord(const Eigen::Vector2d &pt){
  if(pt.x()>global_x_upper_||pt.x()<global_x_lower_||pt.y()>global_y_upper_||pt.y()<global_y_lower_){
    // ROS_ERROR("[CheckCollisionBycoord], coord out of map!!! %f %f",pt.x(),pt.y());
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(pt);
  return gridmap_[index.x() * GLY_SIZE_ + index.y()];
}

uint8_t SDFmap::CheckCollisionBycoord(const double ptx,const double pty){
  if(ptx>global_x_upper_||ptx<global_x_lower_||pty>global_y_upper_||pty<global_y_lower_){
    // ROS_ERROR("[CheckCollisionBycoord], coord out of map!!! %f %f %f",ptx,pty);
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(Eigen::Vector2d(ptx,pty));
  return gridmap_[index.x() * GLY_SIZE_ + index.y()];
}

bool SDFmap::isInGloMap(const Eigen::Vector2d &pt){
  return pt.x() < global_x_upper_ && pt.x() > global_x_lower_ && pt.y() < global_y_upper_ && pt.y() > global_y_lower_;
}

// ============================================================================
// closetPointInMap: 沿射线方向找到地图边界内的最近点
// 功能: 给定一个目标点pt和起始位置pos,如果pt超出地图边界,
//       沿pt-pos方向射线找到与地图边界的交点,返回该交点。
//       用于raycastProcess中将超出地图范围的激光点截断到地图边界内。
// 算法: 参数化射线 pt = pos + t * (pt - pos), t >= 0
//       分别计算与x边界和y边界的交点参数t1, t2,
//       取最小的正t值(第一个遇到的地图边界),返回交点坐标。
// 返回: pos + (min_t - 1e-3) * diff, 轻微缩回避免浮点精度导致越界
// ============================================================================
Eigen::Vector2d SDFmap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos){
  Eigen::Vector2d diff = pt - pos;
  Eigen::Vector2d max_tc = Eigen::Vector2d(global_x_upper_, global_y_upper_) - pos;
  Eigen::Vector2d min_tc = Eigen::Vector2d(global_x_lower_, global_y_lower_) - pos;

  double min_t = 1000000;

  for (int i = 0; i < 2; ++i) {
    if (fabs(diff[i]) > 0) {

      // 沿射线方向与地图上边界(x_upper或y_upper)的交点参数
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      // 沿射线方向与地图下边界(x_lower或y_lower)的交点参数
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  // 取最小正t值对应的交点,减去1e-3保证在地图内部
  return pos + (min_t - 1e-3) * diff;
}



// ============================================================================
// updateESDF2d: 两阶段欧几里得距离变换(Euclidean Distance Transform, EDT)
// 功能: 在机器人周围的局部窗口内计算2D欧几里得符号距离场(ESDF)。
//       障碍物外部距离为正(positive DT),内部距离为负(negative DT)。
// 算法流程:
//   1. 确定ESDF更新窗口: 以机器人为中心,detection_range_为半径
//   2. 第一阶段: 正距离变换(Positive DT)
//      - 先沿y方向: 障碍物网格距离=0,其他网格距离=inf
//      - 再沿x方向: 对y方向的中间结果进行二次变换
//      - 结果乘以grid_interval_恢复实际距离(米)
//   3. 第二阶段: 负距离变换(Negative DT)
//      - 类似正变换,但将Unoccupied/Unknown网格视为边界(距离0)
//      - 障碍物内部得到正距离值
//   4. 合并正负距离: distance_buffer_all_ = positive_DT - negative_DT + grid_interval_
//      最终: 障碍物外部>0, 障碍物内部<0, 边界处≈grid_interval_
//   5. 使用Felzenszwalb距离变换算法(fillESDF): O(n)线性时间
// ============================================================================
void SDFmap::updateESDF2d(){
  Eigen::Vector2i min_esdf(floor(std::max(0.0, odom_pos_.x() - detection_range_ - global_x_lower_)*inv_grid_interval_), floor(std::max(0.0, odom_pos_.y() - detection_range_ - global_y_lower_)*inv_grid_interval_));
  Eigen::Vector2i max_esdf(ceil(std::min(global_x_upper_ - global_x_lower_, odom_pos_.x() + detection_range_ - global_x_lower_)*inv_grid_interval_) - 1,
                           ceil(std::min(global_y_upper_ - global_y_lower_, odom_pos_.y() + detection_range_ - global_y_lower_)*inv_grid_interval_) - 1);

  int update_X_SIZE = max_esdf.x() - min_esdf.x();
  int update_Y_SIZE = max_esdf.y() - min_esdf.y();
  int update_XY_SIZE = (update_X_SIZE + 1) * (update_Y_SIZE + 1);

  std::vector<double> tmp_buffer1_ = std::vector<double>(update_XY_SIZE, 0.0);
  std::vector<double> distance_buffer_ = std::vector<double>(update_XY_SIZE, 0.0);
  std::vector<double> distance_buffer_neg_ = std::vector<double>(update_XY_SIZE, 0.0);
  // distance_buffer_all_ = std::vector<double>(GLXY_SIZE, 0.0);
  /* ========== compute positive DT (distance transform outside the obstacles) ========== */
  for (int x = 0; x <= update_X_SIZE; x++) {
    fillESDF(
        [&](int y) {
          return gridmap_[(x+min_esdf.x()) * GLY_SIZE_ + (y+min_esdf.y())] == Occupied ?
              0.0 :
              std::numeric_limits<double>::max();
        },
        [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; }, 0,
        update_Y_SIZE, update_Y_SIZE+1);
  }
  for (int y = 0; y <= update_Y_SIZE; y++) {
    fillESDF(
      [&](int x) { 
        return tmp_buffer1_[x * update_Y_SIZE + y]; },
              [&](int x, double val) {
                distance_buffer_[x * update_Y_SIZE + y] = grid_interval_ * std::sqrt(val);
              },
              0, update_X_SIZE, update_X_SIZE+1);
  }
  /* ========== compute negative distance inside the obstacles ========== */
  for (int x = 0; x <= update_X_SIZE; x++) {
    fillESDF(
        [&](int y) {
          int state = gridmap_[(x+min_esdf.x()) * GLY_SIZE_ + (y+min_esdf.y())];
          return (state == Unoccupied || state == Unknown) ?
              0.0 :
              std::numeric_limits<double>::max();
        },
        [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; }, 0,
        update_Y_SIZE, update_Y_SIZE+1);
  }
  for (int y = 0; y <= update_Y_SIZE; y++) {
    fillESDF([&](int x) { return tmp_buffer1_[x * update_Y_SIZE + y]; },
              [&](int x, double val) {
                distance_buffer_neg_[x * update_Y_SIZE + y] = grid_interval_ * std::sqrt(val);
              },
              0, update_X_SIZE, update_X_SIZE+1);
  }
  /* ========== combine pos and neg DT ========== */
  for (int x = 0; x < update_X_SIZE; x++)
    for (int y = 0; y < update_Y_SIZE; y++){
        int global_idx = (x + min_esdf.x()) * GLY_SIZE_ + y + min_esdf.y();
        int idx =  x * update_Y_SIZE + y;
        distance_buffer_all_[global_idx] = distance_buffer_[idx];

        if (distance_buffer_neg_[idx] > 0.0)
          distance_buffer_all_[global_idx] += (-distance_buffer_neg_[idx] + grid_interval_);
      }
}

// ============================================================================
// fillESDF: Felzenszwalb距离变换算法(O(n)线性时间)
// 功能: 在一维数组上计算Euclidean Distance Transform的核心算法。
//       使用抛物线最小化技术,通过维护下凸包(lower envelope)来高效计算
//       每个位置到最近边界的最小平方距离。
// 算法原理:
//   阶段1(构建下凸包): 遍历q从start+1到end
//     - 维护数组v[k]记录候选边界点索引, z[k]记录其影响区间边界
//     - 对每个新点q,计算其与当前下凸包最后一个点的交点s
//     - 如果s <= z[k],说明当前下凸包最后一段被"遮蔽",弹出并重试
//     - 否则将q加入下凸包
//   阶段2(查询距离): 遍历q从start到end
//     - 找到q所属的抛物线段(while z[k+1] < q),应用公式计算距离值
//     - f(q) = f(v[k]) + (q - v[k])^2  (平方距离)
// 时间复杂度: O(n), 空间复杂度: O(n)
// 模板参数:
//   F_get_val: 函数 f(idx) -> double, 获取位置idx的初始距离值
//   F_set_val: 函数 f(idx, val) -> void, 设置位置idx的计算结果
// ============================================================================
template <typename F_get_val, typename F_set_val>
void SDFmap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
  int v[dim_size];
  double z[dim_size + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

// ============================================================================
// publish_ESDF: 可视化发布ESDF距离场地图
// 功能: 将distance_buffer_all_中的ESDF距离值以PointCloud2形式发布,
//       用于RViz可视化。每个网格点的intensity值代表该点的ESDF距离,
//       被钳制在[min_dist, max_dist]即[0.0, 5.0]范围内。
//       intensity值越大表示离障碍物越远(在自由空间内越安全)。
// ============================================================================
void SDFmap::publish_ESDF(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 surf_vis;
  const double min_dist = 0.0;
  const double max_dist = 5.0;
  int size = distance_buffer_all_.size();
  for(int i = 1; i< size;i++){
    Eigen::Vector2d coord = gridIndex2coordd(vectornum2gridIndex(i));
    pcl::PointXYZI pt;
    pt.x = coord.x();pt.y = coord.y();pt.z = 0.0;
    pt.intensity = std::max(min_dist, std::min(distance_buffer_all_[i],max_dist));
    cloud_vis.points.push_back(pt);
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "world";
  pub_ESDF_.publish(surf_vis);
}

inline double SDFmap::getDistance(const Eigen::Vector2i& id){
  // if(distance_buffer_all_[Index2Vectornum(id.x(), id.y())]>5){
  //   ROS_ERROR("out of map!!  %d  %d   distance:%f",id.x(),id.y(),distance_buffer_all_[Index2Vectornum(id[0],id[1])]);
  // }
  return distance_buffer_all_[Index2Vectornum(id[0],id[1])];
}

inline double SDFmap::getDistance(const int& idx, const int& idy){
  // if(distance_buffer_all_[Index2Vectornum(idx, idy)]>5){
  //   ROS_ERROR("out of map!!  %d  %d  distance:%f",idx,idy,distance_buffer_all_[Index2Vectornum(idx, idy)]);
  // }
  return distance_buffer_all_[Index2Vectornum(idx, idy)];
}

inline Eigen::Vector2i SDFmap::ESDFcoord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_ - 0.5), 0), GLX_SIZE_ - 1),
         std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_ - 0.5), 0), GLY_SIZE_ - 1);
  return idx;
}

// ============================================================================
// ESDFcoord2gridIndex: ESDF对应的坐标转栅格索引
// 功能: 与coord2gridIndex类似,但用于ESDF查询时的坐标转换。
//       区别在于减去0.5偏移,因为ESDF以栅格顶点(vertex)而非中心(cell center)为基准。
//       这对于双线性插值中正确定位采样点的4个邻居栅格很重要。
// ============================================================================

// ============================================================================
// getDistWithGradBilinear (带梯度): 双线性插值查询ESDF距离及其梯度
// 功能: 在连续坐标pos处通过双线性插值查询ESDF距离值和梯度向量。
//       用于NMPC等需要距离梯度信息的规划算法。
// 算法流程:
//   1. 坐标边界检查: 超出地图范围返回默认值(100),梯度置为0
//   2. 找到距离指定位置最近的左下角栅格idx(使用ESDF顶点坐标)
//   3. 计算pos相对于idx栅格顶点的归一化偏移diff(范围[0,1))
//   4. 读取idx周围2x2区域的4个ESDF距离值
//   5. 双线性插值: 先在x方向插值得到v0/v1,再在y方向插值得到最终距离
//   6. 计算梯度: 对双线性插值公式求偏导,得到x和y方向的梯度分量
// 梯度公式: grad_x = ((1-dy)*(v10-v00) + dy*(v11-v01)) / grid_interval_
//          grad_y = (v1 - v0) / grid_interval_
// ============================================================================
double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

// ============================================================================
// getDistWithGradBilinear (带mindis阈值,带梯度): 优化版双线性插值
// 功能: 与基础版本相同,但额外接收一个mindis阈值参数。
//       如果插值得到的距离dist > mindis,提前返回dist而不计算梯度,
//       因为规划器可能不需要该点的梯度信息,从而节省计算量。
// 参数:
//   mindis: 距离阈值,只有dist <= mindis时才计算梯度
// ============================================================================
double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double& mindis){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  if(dist > mindis){
    return dist;
  }

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

// ============================================================================
// getDistWithGradBilinear (仅距离,不带梯度): 简化版双线性插值
// 功能: 仅通过双线性插值查询ESDF距离值,不计算梯度。
//       适用于只需要距离信息而不需要梯度方向的场景,如碰撞检测。
//       与其他版本相同的双线性插值公式,只是省略了梯度计算部分。
// ============================================================================
double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  return dist; 
}

// ============================================================================
// getDistanceReal: 最近邻查询ESDF距离(无插值)
// 功能: 直接通过栅格索引查询ESDF距离值,不进行任何插值。
//       返回pos所在栅格(使用occupancy map的坐标,非ESDF顶点坐标)的原始距离值。
//       相比双线性插值版本更快速但精度较低,用于对精度要求不高的场景。
//       超出地图范围返回10000(远大于正常距离值)。
// ============================================================================
double SDFmap::getDistanceReal(const Eigen::Vector2d& pos){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    return 10000;
  }
  Eigen::Vector2i idx = coord2gridIndex(pos);
  return distance_buffer_all_[idx.x() * GLY_SIZE_ + idx.y()];
}

// ============================================================================
// getUnkonwnGradBilinear: 双线性插值查询Unknown区域及其梯度
// 功能: 与getDistWithGradBilinear类似,但查询的是Unknown状态而非ESDF距离。
//       用于路径规划中判断某个位置是否位于未知区域及其边缘梯度。
//       双线性插值的4个采样值: gridmap_中Unknown=1, 其他=0
//       返回值在[0,1]之间: 1表示完全在未知区域内,0表示完全在已知区域内。
//       梯度向量指向Unknown区域增加的方向。
// ============================================================================
double SDFmap::getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = coord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = gridmap_[Index2Vectornum(current_idx)]==Unknown?1:0;
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

Eigen::Vector2d SDFmap::get_update_odom(){
  return update_odom_;
}

// ============================================================================
// normalize_angle: 角度归一化到[-PI, PI]范围
// ============================================================================
inline double SDFmap::normalize_angle(double angle){
  if(angle>M_PI) angle -= 2*M_PI;
  if(angle<-M_PI) angle += 2*M_PI;
  return angle;
}

// ============================================================================
// 栅格状态查询函数组: isOccupied / isUnOccupied / isUnknown
// 功能: 查询指定栅格索引处的occupancy状态。
//       每个函数有Eigen::Vector2i和(int idx, int idy)两个重载版本。
//       Occupied:   被障碍物占据
//       Unoccupied: 确认的可行区域(自由空间)
//       Unknown:    尚未探测/状态未知的区域
// ============================================================================
bool SDFmap::isOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Occupied;
}

bool SDFmap::isOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Occupied;
}

bool SDFmap::isUnOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unoccupied;
}

bool SDFmap::isUnOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unoccupied;
}

bool SDFmap::isUnknown(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unknown;
}

bool SDFmap::isUnknown(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unknown;
}

// ============================================================================
// isOccWithSafeDis: 基于ESDF安全距离的碰撞检测
// 功能: 判断指定栅格是否离障碍物太近(ESDF距离 < 安全距离)。
//       不直接查询gridmap_状态,而是通过ESDF距离场判断,
//       这样可以灵活定义安全距离阈值,实现膨胀障碍物的效果。
//       参数: safe_dis为自定义的安全距离阈值(米)
// ============================================================================
bool SDFmap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(index)] < safe_dis;
}

bool SDFmap::isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(idx, idy)] < safe_dis;
}

// ============================================================================
// publish_ESDFGrad: 可视化发布ESDF梯度场(箭头标记)
// 功能: 在RViz中以绿色箭头(Marker::ARROW)可视化ESDF距离场的梯度方向。
//       每隔5个网格采样一次,每个箭头指向ESDF距离增加的方向(即远离障碍物方向)。
//       箭头长度与梯度大小成正比,表示距离场变化的剧烈程度。
// 可视化细节:
//   - 采样步长: 每5个栅格(x,y方向各跳5)
//   - 箭头方向: 从ESDF低值指向高值(指向远离障碍物的方向)
//   - 箭头颜色: 绿色(RGB: 0,1,0),表示安全方向
//   - 箭头缩放: grad.norm()/10, 便于观察
// ============================================================================
void SDFmap::publish_ESDFGrad(){
    visualization_msgs::MarkerArray grad_all;

    Eigen::Vector2i min_cut(0,0);
    Eigen::Vector2i max_cut(GLX_SIZE_-1, GLY_SIZE_-1);

    for (int x = min_cut(0)+2; x < max_cut(0); x+=5)
      for (int y = min_cut(1)+2; y < max_cut(1); y+=5) {
        Eigen::Vector2d pos = gridIndex2coordd(x,y);        
        Eigen::Vector2d d(0.025,0.025);
        pos = pos + d;
        Eigen::Vector2d grad;
        getDistWithGradBilinear(pos,grad);
        visualization_msgs::Marker grad_Marker;
        grad_Marker.header.frame_id = "world";
        grad_Marker.header.stamp = ros::Time::now();
        grad_Marker.ns = "world";
        grad_Marker.type = visualization_msgs::Marker::ARROW;
        grad_Marker.action = visualization_msgs::Marker::ADD;
        grad_Marker.id = (x+1) + (y*100000);
        grad_Marker.pose.position.x = pos[0];
        grad_Marker.pose.position.y = pos[1];
        grad_Marker.pose.position.z = 0.0;
        
        Eigen::Quaterniond Quat;
        Eigen::Vector3d vectorBefore(1, 0, 0);
        Eigen::Vector3d vectorAfter(grad.x(), grad.y(), 0.0);
        Quat = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);

        grad_Marker.pose.orientation.w = Quat.w();
        grad_Marker.pose.orientation.x = Quat.x();
        grad_Marker.pose.orientation.y = Quat.y();
        grad_Marker.pose.orientation.z = Quat.z();

        grad_Marker.scale.x = (grad.norm()+0.01)/10.0;
        if(grad.norm()>1000){
          ROS_INFO("grad error!!!  position: %f  %f    grad:%f  %f   grad.norm(): %f",pos.x(),pos.y(),grad.x(),grad.y(),grad.norm());
        }
        grad_Marker.scale.y = 0.01;
        grad_Marker.scale.z = 0.01;

        grad_Marker.color.r = 0.0f;
        grad_Marker.color.g = 1.0f;
        grad_Marker.color.b = 0.0f;
        grad_Marker.color.a = 1.0;
        grad_Marker.lifetime = ros::Duration(100.0);
        grad_Marker.frame_locked = true;

        grad_all.markers.push_back(grad_Marker);
      }

    pub_gradESDF_.publish(grad_all);

}
