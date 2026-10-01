#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include "../ros_compat/motor_constraint.hpp"

struct JointProbe
{
  std::vector<std::pair<std::string, double>> calls;
  std::string fail;
  bool SetParam(const std::string & key, unsigned axis, double value)
  {
    EXPECT_EQ(axis, 0u);
    calls.emplace_back(key, value);
    return key != fail;
  }
};

TEST(MotorConstraint, PreservesSignedTargetsAndExistingEffortLimit)
{
  for (double target : {-30., -3., 0., 3., 30.}) {
    JointProbe joint;
    EXPECT_TRUE(swerve_bringup::apply_motor_target(joint, target, 200., 30.));
    ASSERT_EQ(joint.calls.size(), 2u);
    EXPECT_EQ(joint.calls[0], std::make_pair(std::string("fmax"), 200.));
    EXPECT_EQ(joint.calls[1], std::make_pair(std::string("vel"), target));
  }
}

TEST(MotorConstraint, RejectsInvalidTargetsRatherThanSilentlyClamping)
{
  for (double target : {31., -31., std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()}) {
    JointProbe joint;
    EXPECT_FALSE(swerve_bringup::apply_motor_target(joint, target, 200., 30.));
    ASSERT_EQ(joint.calls.size(), 1u);
    EXPECT_EQ(joint.calls[0], std::make_pair(std::string("vel"), 0.));
  }
  EXPECT_FALSE(swerve_bringup::valid_motor_target(0., 0., 30.));
  EXPECT_FALSE(swerve_bringup::valid_motor_target(0., 200., 0.));
}

TEST(MotorConstraint, ParameterFailureRequestsZeroAndReportsFailure)
{
  for (const auto & failure : {"fmax", "vel"}) {
    JointProbe joint;
    joint.fail = failure;
    EXPECT_FALSE(swerve_bringup::apply_motor_target(joint, 3., 200., 30.));
    EXPECT_EQ(joint.calls.back(), std::make_pair(std::string("vel"), 0.));
  }
}
