#include <gtest/gtest.h>

#include "point_lio/core/parameters.h"
#include "point_lio/utilities/OdomProcessor.h"

namespace {

OutputFrameParams makeParams() {
  OutputFrameParams out;
  out.base_frame = "base_footprint";
  out.body_to_base_r = {0.0, 0.984807753, 0.173648178,
                        -1.0, 0.0, 0.0,
                        0.0, -0.173648178, 0.984807753};
  out.body_to_base_t = {0.225, 0.0, 0.23};
  return out;
}

}  // namespace

TEST(OdomProcessor, ResolveFromParams) {
  const OutputFrameParams out = makeParams();
  OdomProcessor processor;
  ASSERT_TRUE(processor.resolve(out, rclcpp::get_logger("test")));
  EXPECT_TRUE(processor.ready());

  Eigen::Matrix3d expected;
  expected << 0.0, 0.984807753, 0.173648178,
              -1.0, 0.0, 0.0,
              0.0, -0.173648178, 0.984807753;
  EXPECT_LT((processor.rotation() - expected).cwiseAbs().maxCoeff(), 1e-12);

  const Eigen::Vector3d translation = processor.apply(Eigen::Vector3d::Zero());
  EXPECT_NEAR(translation(0), 0.225, 1e-12);
  EXPECT_NEAR(translation(1), 0.0, 1e-12);
  EXPECT_NEAR(translation(2), 0.23, 1e-12);
}

TEST(OdomProcessor, FillOrderIsRowMajor) {
  // << 按行填充: 前三个数对应 rotation 的第一行。
  OutputFrameParams out;
  out.body_to_base_r = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  out.body_to_base_t = {0.0, 0.0, 0.0};
  OdomProcessor processor;
  ASSERT_TRUE(processor.resolve(out, rclcpp::get_logger("test")));
  EXPECT_DOUBLE_EQ(processor.rotation()(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(processor.rotation()(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(processor.rotation()(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(processor.rotation()(2, 2), 9.0);
}

TEST(OdomProcessor, MissingParamsRejected) {
  OdomProcessor processor;
  OutputFrameParams out;
  out.base_frame = "base_footprint";
  EXPECT_FALSE(processor.resolve(out, rclcpp::get_logger("test")));
  EXPECT_FALSE(processor.ready());

  out.body_to_base_r = {0.0, 0.0, 0.0};
  EXPECT_FALSE(processor.resolve(out, rclcpp::get_logger("test")));
  EXPECT_FALSE(processor.ready());
}
