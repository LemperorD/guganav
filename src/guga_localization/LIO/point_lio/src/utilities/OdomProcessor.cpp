#include "point_lio/utilities/OdomProcessor.h"

#include "point_lio/utilities/common_lib.h"

bool OdomProcessor::resolve(const OutputFrameParams& out,
                            const rclcpp::Logger& logger) {
  if (ready_) {
    return true;
  }
  if (out.body_to_base_r.size() != 9 || out.body_to_base_t.size() != 3) {
    RCLCPP_ERROR(logger,
                 "缺少 output_frame.body_to_base_r/t, 无法解析 %s <- body",
                 out.base_frame.c_str());
    return false;
  }

  rotation_ << out.body_to_base_r[0], out.body_to_base_r[1],
      out.body_to_base_r[2], out.body_to_base_r[3], out.body_to_base_r[4],
      out.body_to_base_r[5], out.body_to_base_r[6], out.body_to_base_r[7],
      out.body_to_base_r[8];
  translation_ << out.body_to_base_t[0], out.body_to_base_t[1],
      out.body_to_base_t[2];
  ready_ = true;
  RCLCPP_INFO(logger, "odom 输出使用显式参数 %s <- body",
              out.base_frame.c_str());
  return true;
}
