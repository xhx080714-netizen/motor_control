#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "sand_rake_control/diff_drive_kinematics.hpp"

using sand_rake_control::DiffDriveKinematics;

TEST(DiffDriveKinematics, RejectsInvalidGeometry)
{
  EXPECT_THROW(
    DiffDriveKinematics(0.0, 0.5, 7.5, 1500.0, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    DiffDriveKinematics(0.1, 0.0, 7.5, 1500.0, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    DiffDriveKinematics(0.1, 0.5, 7.5, 1500.0, 1501.0),
    std::invalid_argument);
}

TEST(DiffDriveKinematics, RejectsNonFiniteParameters)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    DiffDriveKinematics(nan, 0.5, 7.5, 1500.0, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    DiffDriveKinematics(0.1, infinity, 7.5, 1500.0, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    DiffDriveKinematics(0.1, 0.5, nan, 1500.0, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    DiffDriveKinematics(0.1, 0.5, 7.5, infinity, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    DiffDriveKinematics(0.1, 0.5, 7.5, 1500.0, nan),
    std::invalid_argument);
}

TEST(DiffDriveKinematics, StraightCommandMatchesSides)
{
  DiffDriveKinematics kinematics(0.1, 0.5, 1.0, 1000.0, 0.0);
  const auto sides = kinematics.inverse(0.2, 0.0);
  EXPECT_DOUBLE_EQ(sides.left_mps, 0.2);
  EXPECT_DOUBLE_EQ(sides.right_mps, 0.2);
}

TEST(DiffDriveKinematics, LeftTurnMakesRightSideFaster)
{
  DiffDriveKinematics kinematics(0.1, 0.5, 1.0, 1000.0, 0.0);
  const auto sides = kinematics.inverse(0.1, 0.4);
  EXPECT_LT(sides.left_mps, sides.right_mps);
}

TEST(DiffDriveKinematics, RpmConversionRoundTrips)
{
  DiffDriveKinematics kinematics(0.134665, 0.53907, 7.5, 1500.0, 30.0);
  const double input = 0.15;
  const double recovered =
    kinematics.motor_rpm_to_linear(kinematics.linear_to_motor_rpm(input));
  EXPECT_NEAR(recovered, input, 1.0e-12);
}

TEST(DiffDriveKinematics, ScalesAllWheelsToMotorLimit)
{
  DiffDriveKinematics kinematics(0.1, 0.5, 1.0, 100.0, 30.0);
  const auto rpm = kinematics.command_to_motor_rpm(10.0, 2.0);
  EXPECT_LE(rpm.left_front, 100.0);
  EXPECT_LE(rpm.right_front, 100.0);
  EXPECT_DOUBLE_EQ(rpm.left_front, rpm.left_rear);
  EXPECT_DOUBLE_EQ(rpm.right_front, rpm.right_rear);
}

TEST(DiffDriveKinematics, MatchesFollowIouRpmQuantization)
{
  DiffDriveKinematics kinematics(0.134665, 0.53907, 7.5, 1500.0, 30.0);

  const auto rounded_to_zero = kinematics.command_to_motor_rpm(
    kinematics.motor_rpm_to_linear(0.5), 0.0);
  EXPECT_DOUBLE_EQ(rounded_to_zero.left_front, 0.0);

  const auto clamped_to_minimum = kinematics.command_to_motor_rpm(
    kinematics.motor_rpm_to_linear(1.5), 0.0);
  EXPECT_DOUBLE_EQ(clamped_to_minimum.left_front, 30.0);

  const auto half_to_even = kinematics.command_to_motor_rpm(
    kinematics.motor_rpm_to_linear(31.5), 0.0);
  EXPECT_DOUBLE_EQ(half_to_even.left_front, 32.0);

  const auto negative = kinematics.command_to_motor_rpm(
    kinematics.motor_rpm_to_linear(-1.5), 0.0);
  EXPECT_DOUBLE_EQ(negative.left_front, -30.0);
}
