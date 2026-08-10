# MincoTrajectory.msg
# 用于在 MINCO Smoother 和 MPC Controller 之间传输多项式轨迹
#
# 轨迹表示：每段是一个三次多项式 (degree=3)
# p(t) = c0 + c1*t + c2*t^2 + c3*t^3
# 
# 系数存储格式：
# - 2D 轨迹：每段有 2 维 (x, y)，每维有 4 个系数 (c0, c1, c2, c3)
# - 展平顺序：[段0_x_c0, 段0_x_c1, 段0_x_c2, 段0_x_c3, 
#              段0_y_c0, 段0_y_c1, 段0_y_c2, 段0_y_c3,
#              段1_x_c0, ...]
# - 总长度：N * 2 * 4 = N * 8，其中 N 是段数

# 消息头，包含时间戳和坐标系
std_msgs/Header header

# 轨迹唯一标识符（用于匹配请求/响应，或检测更新）
uint32 trajectory_id

# 轨迹起始绝对时间（ROS 时间戳）
# Controller 使用此时间计算当前相对时间 t = now - start_time
builtin_interfaces/Time start_time

# 每段轨迹的持续时间数组 (N 段)
float64[] durations

# 展平的多项式系数数组 (N * 2 * 4)
# 系数顺序：对于每段，先 x 维度的 [c3, c2, c1, c0]，再 y 维度的 [c3, c2, c1, c0]
# 注意：系数从高次到低次存储，与 Trajectory<3>::Piece 的 coeffs_ 一致
float64[] coefficients
