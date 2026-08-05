/**
 * @file traj_representation.h
 * @brief 轨迹表示数据结构定义
 *
 * 本文件定义了前端路径规划模块中的核心数据结构:
 * - PathNode: 用于混合A*（Hybrid A*）等搜索算法中的路径节点，包含离散索引和连续状态
 * - FlatTrajData: 平坦化轨迹数据，存储经过时间参数化和平坦输出的完整轨迹信息
 *
 * 设计意图:
 * - 将栅格地图中的离散搜索（整数索引）与连续空间中的运动规划（浮点坐标+偏航角）解耦
 * - 通过 FlatTrajData 将前端的参考路径传递给后端优化器（如 NMPC/轨迹优化）
 */

#ifndef _TRAJ_REPRESENTATION_H
#define _TRAJ_REPRESENTATION_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>

// 节点状态宏定义：使用单字符标记，比枚举更轻量
#define IN_CLOSE_SET 'a'  // 节点已扩展完毕，在closed集合中
#define IN_OPEN_SET 'b'   // 节点等待扩展，在open集合中
#define NOT_EXPAND 'c'    // 节点尚未被扩展

// 无穷大常量（注意: 1>>30 = 0，这里可能是一个遗留bug，实际应使用更大的值或 std::numeric_limits）
#define inf 1 >> 30

/**
 * @class PathNode
 * @brief 混合A*搜索算法中的通用路径节点
 *
 * 设计意图:
 * - 在栅格地图中执行运动基元搜索时，每个节点需同时记录离散栅格坐标（用于查表）
 *   和连续状态（用于运动学约束验证）
 * - 支持混合A*的关键特性：f = g + h + penalty，其中penalty可用于碰撞惩罚或曲率惩罚
 *
 * 关键数据成员:
 * - index: 二维栅格索引（整数），用于哈希表快速查找和状态离散化
 * - yaw_idx: 偏航角的离散化索引（在混合A*中，同一栅格可能有多个不同朝向的节点）
 * - state: 连续状态 [x, y, theta]，用于精确的运动学计算
 * - g_score: 从起点到当前节点已消耗的代价
 * - f_score: 总估计代价 f = g + h（启发式）
 * - penalty_score: 惩罚代价，用于引入软约束（如靠近障碍物的惩罚）
 * - input: 控制输入 [steer_angle, arc_length]，记录到达此节点的控制量
 * - parent: 父节点指针，用于路径回溯
 */
class PathNode {
public:
  /* -------------------- */
  Eigen::Vector2i index;     // 二维栅格索引 [x_idx, y_idx]，用于状态空间离散化
  int yaw_idx;               // 偏航角离散索引，同一栅格可有多朝向节点
  /* --- 连续状态为 (x, y, theta)，其中theta为朝向角 --- */
  Eigen::Vector3d state;     // 连续状态 [x, y, theta]（world坐标系）
  double g_score, f_score;   // g: 实际代价，f = g + h: 估计总代价
  double penalty_score;      // 惩罚项：用于加入软约束（碰撞风险、曲率等）
  /* 控制输入包含转向角和弧长 */
  Eigen::Vector2d input;     // 控制输入 [steer, arc_length]（Dubins/Reeds-Shepp路径段）
  PathNode* parent;          // 父节点指针，用于在搜索完成后回溯重建路径
  // 三种状态: 未扩展、在关闭集、在开放集（对应A*算法的三状态模型）
  char node_state;           // 节点当前状态（IN_OPEN_SET / IN_CLOSE_SET / NOT_EXPAND）
  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;  // 初始化为未扩展状态
  }
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;

/**
 * @struct FlatTrajData
 * @brief 时间参数化的平坦轨迹数据结构
 *
 * 设计意图:
 * - 前端通过 JPS/混合A* 生成几何路径后，使用梯形速度剖面进行时间参数化
 * - FlatTrajData 封装了等时间间隔采样的轨迹点，作为后端NMPC或轨迹优化的输入
 * - "平坦"指轨迹被展开为一维的时间序列（类似差分平坦系统中的平坦输出概念）
 *
 * 主要数据成员:
 * - UnOccupied_traj_pts: 等时间采样的轨迹点，每个点为 [yaw, s, t]，其中
 *   yaw = 期望偏航角，s = 沿路径的弧长，t = 时间戳
 * - UnOccupied_positions: 对应的笛卡尔坐标 [x, y, yaw]
 * - start_state / final_state: 起止时刻的 pva（位置/速度/加速度）状态矩阵 (2x3)
 * - if_cut: 标记轨迹是否因为长度限制被截断
 */
struct FlatTrajData{

  // 均匀采样后的所有初始值: yaw, s, t
  // yaw: 期望航向角, s: 沿路径弧长进度, t: 采样时间点
  std::vector<Eigen::Vector3d> UnOccupied_traj_pts;
  double UnOccupied_initT;                           // 采样时间间隔 (delta_t)
  std::vector<Eigen::Vector3d> UnOccupied_positions; // 采样位置 [x, y, yaw]

  Eigen::MatrixXd start_state;   // 起始状态 pva: [位置(含yaw), 速度, 加速度] 每列3维
  Eigen::MatrixXd final_state;   // 终止状态 pva: 用于轨迹拼接的末端边界条件 (2行3列)

  Eigen::Vector3d start_state_XYTheta;  // 起始位置+朝向 [x, y, yaw]
  Eigen::Vector3d final_state_XYTheta;  // 终止/截断点位置+朝向 [x, y, yaw]
  bool if_cut;                          // 标记: 轨迹是否因为超过 trajCutLength_ 而被截断

  /// 调试输出函数：打印 FlatTrajData 中所有字段的内容
  void printFlatTrajData() {
    std::cout << "UnOccupied_traj_pts:" << std::endl;
    for (const auto& pt : UnOccupied_traj_pts) {
      std::cout << pt.transpose() << std::endl;
    }
    std::cout << "UnOccupied_initT: " << UnOccupied_initT << std::endl;

    std::cout << "start_state:" << std::endl;
    std::cout << start_state << std::endl;
    std::cout << "final_state:" << std::endl;
    std::cout << final_state << std::endl;
    std::cout << "start_state_XYTheta: " << start_state_XYTheta.transpose() << std::endl;
    std::cout << "final_state_XYTheta: " << final_state_XYTheta.transpose() << std::endl;
    std::cout << "if_cut: " << if_cut << std::endl;
  }
};




#endif
