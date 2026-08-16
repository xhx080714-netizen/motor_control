#include "sand_rake_control/diff_drive_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sand_rake_control
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

DiffDriveKinematics::DiffDriveKinematics(
  double wheel_radius_m,
  double effective_track_width_m,
  double gear_ratio,
  double max_motor_rpm,
  double min_motor_rpm)
: wheel_radius_m_(wheel_radius_m),
  effective_track_width_m_(effective_track_width_m),
  gear_ratio_(gear_ratio),
  max_motor_rpm_(max_motor_rpm),
  min_motor_rpm_(min_motor_rpm)
{
  if (wheel_radius_m_ <= 0.0) {
    throw std::invalid_argument("wheel_radius_m must be positive");
  }
  if (effective_track_width_m_ <= 0.0) {
    throw std::invalid_argument("effective_track_width_m must be positive");
  }
  if (gear_ratio_ <= 0.0) {
    throw std::invalid_argument("gear_ratio must be positive");
  }
  if (max_motor_rpm_ <= 0.0) {
    throw std::invalid_argument("max_motor_rpm must be positive");
  }
  if (min_motor_rpm_ < 0.0 || min_motor_rpm_ > max_motor_rpm_) {
    throw std::invalid_argument(
            "min_motor_rpm must be in [0, max_motor_rpm]");
  }

}

SideSpeeds DiffDriveKinematics::inverse(
  double linear_mps, double angular_rps) const
{
  // Same equations as follow_iou_c/src/diff_drive.c.
  return {
    linear_mps - angular_rps * effective_track_width_m_ * 0.5,
    linear_mps + angular_rps * effective_track_width_m_ * 0.5};
}

BodyTwist DiffDriveKinematics::forward(
  double left_mps, double right_mps) const
{
  return {
    (left_mps + right_mps) * 0.5,
    (right_mps - left_mps) / effective_track_width_m_};
}

double DiffDriveKinematics::linear_to_motor_rpm(double linear_mps) const
{
  return linear_mps * 60.0 * gear_ratio_ / (kTwoPi * wheel_radius_m_);
}

double DiffDriveKinematics::motor_rpm_to_linear(double motor_rpm) const
{
  return kTwoPi * wheel_radius_m_ * motor_rpm / (60.0 * gear_ratio_);
}

WheelRpmValues DiffDriveKinematics::command_to_motor_rpm(
  double linear_mps, double angular_rps) const
{
  const auto sides = inverse(linear_mps, angular_rps);
  const double left_rpm = quantize_motor_rpm(
    linear_to_motor_rpm(sides.left_mps));
  const double right_rpm = quantize_motor_rpm(
    linear_to_motor_rpm(sides.right_mps));

  return {left_rpm, left_rpm, right_rpm, right_rpm};
}

double DiffDriveKinematics::quantize_motor_rpm(double motor_rpm) const
{
  // Match follow_iou_c/src/diff_drive.c and include/pyround.h: Python 3
  // round-half-to-even first, then clamp nonzero commands to min/max rpm.
  const double lower = std::floor(motor_rpm);
  const double fraction = motor_rpm - lower;
  double rounded = lower;
  if (fraction > 0.5 ||
    (fraction == 0.5 && std::fmod(std::abs(lower), 2.0) == 1.0))
  {
    rounded = lower + 1.0;
  }
  if (rounded == 0.0) {
    return 0.0;
  }
  const double sign = rounded > 0.0 ? 1.0 : -1.0;
  const double magnitude = std::clamp(
    std::abs(rounded), min_motor_rpm_, max_motor_rpm_);
  return sign * magnitude;
}

BodyTwist DiffDriveKinematics::motor_rpm_to_body_twist(
  const WheelRpmValues & rpm) const
{
  const double left_rpm = (rpm.left_front + rpm.left_rear) * 0.5;
  const double right_rpm = (rpm.right_front + rpm.right_rear) * 0.5;
  return forward(
    motor_rpm_to_linear(left_rpm),
    motor_rpm_to_linear(right_rpm));
}

double DiffDriveKinematics::wheel_radius_m() const
{
  return wheel_radius_m_;
}

double DiffDriveKinematics::effective_track_width_m() const
{
  return effective_track_width_m_;
}

double DiffDriveKinematics::gear_ratio() const
{
  return gear_ratio_;
}

double DiffDriveKinematics::max_motor_rpm() const
{
  return max_motor_rpm_;
}

double DiffDriveKinematics::min_motor_rpm() const
{
  return min_motor_rpm_;
}

}  // namespace sand_rake_control
