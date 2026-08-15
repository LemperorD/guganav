// Copyright 2026 Guga Robotics
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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace fake_vel_transform
{

TEST(FakeVelTransformTest, SelectsYawForCurrentChassisMode)
{
  EXPECT_DOUBLE_EQ(FakeVelTransform::selectVelocityYawDiff(chassisFollowed, 0.3, 1.2), 0.3);
  EXPECT_DOUBLE_EQ(FakeVelTransform::selectVelocityYawDiff(littleTES, 0.3, 1.2), 1.2);
}

TEST(FakeVelTransformTest, RotatesFakeFrameVelocityIntoChassisFrame)
{
  geometry_msgs::msg::Twist input;
  input.linear.x = 1.0;

  const auto output = FakeVelTransform::rotateVelocity(input, std::acos(-1.0) / 2.0);

  EXPECT_NEAR(output.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(output.linear.y, -1.0, 1e-12);
}

}  // namespace fake_vel_transform
