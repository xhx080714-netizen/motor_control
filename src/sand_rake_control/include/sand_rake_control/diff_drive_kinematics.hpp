#ifndef SAND_RAKE_CONTROL__DIFF_DRIVE_KINEMATICS_HPP_
#define SAND_RAKE_CONTROL__DIFF_DRIVE_KINEMATICS_HPP_

namespace sand_rake_control
{

struct SideSpeeds
{
  double left_mps{0.0};
  double right_mps{0.0};
};

struct BodyTwist
{
  double linear_mps{0.0};
  double angular_rps{0.0};
};

struct WheelRpmValues
{
  double left_front{0.0};
  double left_rear{0.0};
  double right_front{0.0};
  double right_rear{0.0};
};

class DiffDriveKinematics
{
public:
  DiffDriveKinematics(
    double wheel_radius_m,
    double effective_track_width_m,
    double gear_ratio,
    double max_motor_rpm,
    double min_motor_rpm);

  SideSpeeds inverse(double linear_mps, double angular_rps) const;
  BodyTwist forward(double left_mps, double right_mps) const;
  double linear_to_motor_rpm(double linear_mps) const;
  double motor_rpm_to_linear(double motor_rpm) const;
  WheelRpmValues command_to_motor_rpm(
    double linear_mps, double angular_rps) const;
  BodyTwist motor_rpm_to_body_twist(const WheelRpmValues & rpm) const;

  double wheel_radius_m() const;
  double effective_track_width_m() const;
  double gear_ratio() const;
  double max_motor_rpm() const;
  double min_motor_rpm() const;

private:
  double quantize_motor_rpm(double motor_rpm) const;

  double wheel_radius_m_;
  double effective_track_width_m_;
  double gear_ratio_;
  double max_motor_rpm_;
  double min_motor_rpm_;
};

}  // namespace sand_rake_control

#endif  // SAND_RAKE_CONTROL__DIFF_DRIVE_KINEMATICS_HPP_
