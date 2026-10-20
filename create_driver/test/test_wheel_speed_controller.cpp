// Copyright 2026 GamerSocks

#include <gtest/gtest.h>

#include "create_driver/wheel_speed_controller.h"

using create_driver::WheelSpeedController;

namespace
{
const WheelSpeedController::Gains GAINS{0.3, 3.0, 0.1};
constexpr double DT = 0.02;

// Wheels that reach a fixed fraction of their command, measured one step later.
struct Plant
{
  double left_gain;
  double right_gain;
  double left = 0.0;
  double right = 0.0;

  void apply(const std::pair<double, double> & command)
  {
    left = left_gain * command.first;
    right = right_gain * command.second;
  }
};
}  // namespace

TEST(WheelSpeedController, ConvergesForSlowAndFastWheel)
{
  WheelSpeedController controller(GAINS);
  Plant plant{0.93, 1.12};
  for (int step = 0; step < 300; ++step) {
    plant.apply(controller.update(0.25, 0.25, plant.left, plant.right, true, DT));
  }
  EXPECT_NEAR(plant.left, 0.25, 1e-3);
  EXPECT_NEAR(plant.right, 0.25, 1e-3);
}

TEST(WheelSpeedController, CorrectionIsLimited)
{
  WheelSpeedController controller(GAINS);
  Plant plant{0.5, 1.0};
  std::pair<double, double> command;
  for (int step = 0; step < 300; ++step) {
    command = controller.update(0.25, 0.25, plant.left, plant.right, true, DT);
    plant.apply(command);
  }
  EXPECT_NEAR(command.first, 0.25 + GAINS.max_correction, 1e-9);
}

TEST(WheelSpeedController, StopAndReversalRestartWithoutCorrection)
{
  WheelSpeedController controller(GAINS);
  Plant plant{0.9, 0.9};
  for (int step = 0; step < 100; ++step) {
    plant.apply(controller.update(0.2, 0.2, plant.left, plant.right, true, DT));
  }
  EXPECT_EQ(controller.update(0.0, 0.0, plant.left, plant.right, true, DT), std::make_pair(0.0, 0.0));
  // Reversing directly also discards the correction learned for the other direction.
  controller.update(0.2, 0.2, 0.2, 0.2, true, DT);
  EXPECT_EQ(controller.update(-0.2, -0.2, 0.2, 0.2, true, DT), std::make_pair(-0.2, -0.2));
}

TEST(WheelSpeedController, CommandNeverOpposesTarget)
{
  WheelSpeedController controller(GAINS);
  // Measured speed far above a small target, as at the end of a ramp-down.
  for (int step = 0; step < 50; ++step) {
    const auto command = controller.update(0.01, -0.01, 0.2, -0.2, true, DT);
    EXPECT_GE(command.first, 0.0);
    EXPECT_LE(command.second, 0.0);
  }
}

TEST(WheelSpeedController, InvalidMeasurementPassesTargetsThrough)
{
  WheelSpeedController controller(GAINS);
  EXPECT_EQ(controller.update(0.25, -0.1, 0.0, 0.0, false, DT), std::make_pair(0.25, -0.1));
  EXPECT_EQ(controller.update(0.25, -0.1, 0.0, 0.0, true, 0.0), std::make_pair(0.25, -0.1));
}
