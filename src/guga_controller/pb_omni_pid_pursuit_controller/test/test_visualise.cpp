#include "pb_omni_pid_pursuit_controller/visualise.hpp"
#include "gtest/gtest.h"

// createCarrotMsg 提取 carrot_pose 的 x,y 坐标，z 固定为 0.01
TEST(VisualiseTest, CreateCarrotMsg_CopiesXYAndSetsZ) {
  geometry_msgs::msg::PoseStamped carrot;
  carrot.header.frame_id = "base_link";
  carrot.pose.position.x = 3.0;
  carrot.pose.position.y = -1.5;
  carrot.pose.position.z = 5.0;  // should be ignored

  auto msg = visualization_helper::createCarrotMsg(carrot);

  EXPECT_EQ(msg->header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(msg->point.x, 3.0);
  EXPECT_DOUBLE_EQ(msg->point.y, -1.5);
  EXPECT_DOUBLE_EQ(msg->point.z, 0.01);
}
