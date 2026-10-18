// Copyright 2026 GamerSocks

#include "create_driver/wheel_speed_controller.h"

namespace create_driver
{

WheelSpeedController::WheelSpeedController(const Gains & gains)
{
  setGains(gains);
}

void WheelSpeedController::setGains(const Gains & gains)
{
  control_toolbox::AntiWindupStrategy antiwindup;
  antiwindup.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;
  antiwindup.i_max = gains.max_correction;
  antiwindup.i_min = -gains.max_correction;
  for (Wheel * wheel : {&left_, &right_}) {
    wheel->pid.set_gains(gains.p, gains.i, 0.0, gains.max_correction, -gains.max_correction, antiwindup);
  }
}

void WheelSpeedController::reset()
{
  for (Wheel * wheel : {&left_, &right_}) {
    wheel->pid.reset();
    wheel->last_target = 0.0;
  }
}

std::pair<double, double> WheelSpeedController::update(
  double target_left, double target_right, double measured_left, double measured_right,
  bool measurement_valid, double dt)
{
  return {
    updateWheel(left_, target_left, measured_left, measurement_valid, dt),
    updateWheel(right_, target_right, measured_right, measurement_valid, dt)};
}

double WheelSpeedController::updateWheel(
  Wheel & wheel, double target, double measured, bool measurement_valid, double dt)
{
  // A stop or reversal invalidates the learned offset, which depends on direction.
  const bool restart = target == 0.0 || target * wheel.last_target < 0.0;
  wheel.last_target = target;
  if (restart || !measurement_valid || dt <= 0.0) {
    wheel.pid.reset();
    return target;
  }
  return target + wheel.pid.compute_command(target - measured, dt);
}

}  // namespace create_driver
