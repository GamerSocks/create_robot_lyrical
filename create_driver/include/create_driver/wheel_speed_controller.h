// Copyright 2026 GamerSocks

#ifndef CREATE_DRIVER__WHEEL_SPEED_CONTROLLER_H_
#define CREATE_DRIVER__WHEEL_SPEED_CONTROLLER_H_

#include <utility>

#include "control_toolbox/pid.hpp"

namespace create_driver
{

/**
 * Per-wheel speed correction on top of the Create firmware's own wheel control.
 *
 * Each wheel command is its target speed plus a PI correction of the error between
 * target and encoder-measured speed, limited to +/- max_correction. Integration pauses
 * while the correction saturates and restarts from zero when the wheel stops or reverses.
 * The command never points against the target direction.
 */
class WheelSpeedController
{
public:
  struct Gains
  {
    double p = 0.0;
    double i = 0.0;
    double max_correction = 0.0;  // m/s
  };

  /// Without gains the correction stays zero until setGains is called.
  WheelSpeedController() = default;
  explicit WheelSpeedController(const Gains & gains);

  void setGains(const Gains & gains);
  void reset();

  /// Wheel commands (left, right) in m/s; without a valid measurement the targets pass through.
  std::pair<double, double> update(
    double target_left, double target_right, double measured_left, double measured_right,
    bool measurement_valid, double dt);

private:
  struct Wheel
  {
    control_toolbox::Pid pid;
    double last_target = 0.0;
  };

  static double updateWheel(Wheel & wheel, double target, double measured, bool measurement_valid, double dt);

  Wheel left_;
  Wheel right_;
};

}  // namespace create_driver

#endif  // CREATE_DRIVER__WHEEL_SPEED_CONTROLLER_H_
