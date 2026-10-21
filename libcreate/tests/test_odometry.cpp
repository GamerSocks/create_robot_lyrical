// Copyright 2026 GamerSocks

#include "create/create.h"
#include "create/types.h"

#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <tuple>

namespace create {
// Drive the real integration code with validated sensor fields and exact times.
class CreateTestAccess {
public:
  static void yaw(Create &robot, float angle) { robot.pose.yaw = angle; }
  static void sample(Create &robot, uint16_t left, uint16_t right, double seconds)
  {
    for (const auto value : {std::make_pair(ID_LEFT_ENC, left), std::make_pair(ID_RIGHT_ENC, right)}) {
      auto packet = robot.data->getPacket(value.first);
      packet->setDataToValidate(value.second);
      packet->validate();
    }
    const auto time = std::chrono::steady_clock::time_point{} +
                      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(seconds));
    robot.onDataAt(time);
  }
};
}

namespace {
const double METRES_PER_TICK =
    create::RobotModel::CREATE_2.getWheelDiameter() * create::util::PI / create::util::V_3_TICKS_PER_REV;
using create::CreateTestAccess;

create::OdometryCalibration noiseCalibration(const create::Create &robot)
{
  auto calibration = robot.getOdometryCalibration();
  calibration.left_noise_k = .01;
  calibration.right_noise_k = .02;
  calibration.left_noise_q0 = 1e-6;
  calibration.right_noise_q0 = 4e-6;
  calibration.vx_variance_floor = 0.0;
  calibration.yaw_rate_variance_floor = 0.0;
  return calibration;
}
}

TEST(CreateOdometry, SignedEncoderWrapsBothWays)
{
  for (const auto &counts : {std::make_tuple(65530, 5, 11), std::make_tuple(5, 65530, -11),
                             std::make_tuple(65535, 0, 1), std::make_tuple(0, 65535, -1)}) {
    create::Create robot(create::RobotModel::CREATE_2, false);
    CreateTestAccess::sample(robot, std::get<0>(counts), std::get<0>(counts), 1.0);
    ASSERT_FALSE(robot.getOdometryState().valid);
    CreateTestAccess::sample(robot, std::get<1>(counts), std::get<1>(counts), 1.1);
    const auto state = robot.getOdometryState();
    ASSERT_TRUE(state.valid);
    EXPECT_NEAR(state.pose.x, std::get<2>(counts) * METRES_PER_TICK, 1e-7);
    EXPECT_NEAR(state.velocity.x, std::get<2>(counts) * METRES_PER_TICK / .1, 1e-6);
  }
}

TEST(CreateOdometry, VelocityWindowHandlesIrregularIntervalsAndDroppedPackets)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(3);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 10, 10, 1.01);
  CreateTestAccess::sample(robot, 30, 30, 1.03);
  CreateTestAccess::sample(robot, 130, 130, 1.13); // missed samples retain cumulative ticks
  auto state = robot.getOdometryState();
  EXPECT_NEAR(state.window_duration, .13, 1e-9);
  EXPECT_NEAR(state.velocity.x, 1000 * METRES_PER_TICK, 1e-6);
  CreateTestAccess::sample(robot, 130, 130, 1.15); // removes the first interval from the window
  state = robot.getOdometryState();
  EXPECT_NEAR(state.window_duration, .14, 1e-9);
  EXPECT_NEAR(state.velocity.x, 120 * METRES_PER_TICK / .14, 1e-6);
  EXPECT_NEAR(state.pose.x, 130 * METRES_PER_TICK, 1e-6);
}

TEST(CreateOdometry, CurvesAndTurnsIntegrateExactArcs)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(1);
  const double axle = create::RobotModel::CREATE_2.getAxleLength();
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 90, 110, 1.1);
  auto state = robot.getOdometryState();
  EXPECT_NEAR(state.pose.x, 0.0, 1e-7);
  EXPECT_NEAR(state.pose.yaw, 20 * METRES_PER_TICK / axle, 1e-7);
  EXPECT_NEAR(state.velocity.yaw, 20 * METRES_PER_TICK / axle / .1, 1e-6);
  const double oldYaw = state.pose.yaw;
  CreateTestAccess::sample(robot, 110, 150, 1.2);
  state = robot.getOdometryState();
  const double yawDelta = 20 * METRES_PER_TICK / axle;
  const double radius = 30 * METRES_PER_TICK / yawDelta;
  EXPECT_NEAR(state.pose.x, radius * (sin(oldYaw + yawDelta) - sin(oldYaw)), 1e-6);
  EXPECT_NEAR(state.pose.y, -radius * (cos(oldYaw + yawDelta) - cos(oldYaw)), 1e-6);
  EXPECT_NEAR(state.velocity.x, 30 * METRES_PER_TICK / .1, 1e-6);
}

TEST(CreateOdometry, GapInvalidatesWithoutInventingMotion)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 110, 1.1);
  const auto before = robot.getOdometryState();
  CreateTestAccess::sample(robot, 1010, 1010, 2.2);
  auto state = robot.getOdometryState();
  EXPECT_FALSE(state.valid);
  EXPECT_FLOAT_EQ(state.pose.x, before.pose.x);
  CreateTestAccess::sample(robot, 1020, 1020, 2.3);
  state = robot.getOdometryState();
  EXPECT_TRUE(state.valid);
  EXPECT_NEAR(state.pose.x, 20 * METRES_PER_TICK, 1e-7);
  EXPECT_NEAR(state.velocity.x, 10 * METRES_PER_TICK / .1, 1e-6);
}

TEST(CreateOdometry, DistanceScalesAndAxleLengthApply)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(1);
  auto calibration = robot.getOdometryCalibration();
  calibration.left_distance_scale = .5;
  calibration.right_distance_scale = .25;
  calibration.axle_length = .30;
  robot.setOdometryCalibration(calibration);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 120, 140, 1.1);
  EXPECT_NEAR(robot.getPose().x, 10 * METRES_PER_TICK, 1e-8);
  EXPECT_NEAR(robot.getPose().yaw, 0.0, 1e-8);
  CreateTestAccess::sample(robot, 100, 180, 1.2);
  EXPECT_NEAR(robot.getPose().yaw, 20 * METRES_PER_TICK / .30, 1e-7);
}

TEST(CreateOdometryCovariance, VelocityCovarianceMatchesWheelNoiseModel)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  auto calibration = noiseCalibration(robot);
  calibration.wheel_error_correlation = .25;
  robot.setOdometryCalibration(calibration);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 120, 1.1);
  CreateTestAccess::sample(robot, 115, 135, 1.3);
  const auto state = robot.getOdometryState();
  const double qr = .02 * 35 * METRES_PER_TICK + 4e-6;
  const double ql = .01 * 15 * METRES_PER_TICK + 1e-6;
  const double c = .25 * std::sqrt(qr * ql);
  const double b = calibration.axle_length;
  EXPECT_NEAR(state.velocity.covariance[0], (qr + ql + 2 * c) / (4 * .09), 1e-9);
  EXPECT_NEAR(state.velocity.covariance[8], (qr + ql - 2 * c) / (b * b * .09), 1e-7);
  EXPECT_NEAR(state.velocity.covariance[2], (qr - ql) / (2 * b * .09), 1e-8);
  EXPECT_EQ(state.velocity.covariance[2], state.velocity.covariance[6]);
}

TEST(CreateOdometryCovariance, ReverseTravelDoesNotCancelUncertainty)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(2);
  robot.setOdometryCalibration(noiseCalibration(robot));
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 120, 1.1);
  CreateTestAccess::sample(robot, 100, 100, 1.2);
  EXPECT_NEAR(robot.getVel().x, 0, 1e-8);
  const double qr = .02 * 40 * METRES_PER_TICK + 4e-6;
  const double ql = .01 * 20 * METRES_PER_TICK + 1e-6;
  EXPECT_NEAR(robot.getVel().covariance[0], (qr + ql) / .16, 1e-8);
}

TEST(CreateOdometryCovariance, PoseCovarianceMatchesNumericalJacobian)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  auto calibration = noiseCalibration(robot);
  calibration.wheel_error_correlation = .3;
  calibration.left_noise_q0 = calibration.right_noise_q0 = 0;
  calibration.yaw_orientation_variance_floor = 0;
  robot.setOdometryCalibration(calibration);
  CreateTestAccess::yaw(robot, 1.2f);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 40, 400, 1.1);
  const double R = 400 * METRES_PER_TICK, L = 40 * METRES_PER_TICK, b = calibration.axle_length;
  const auto increment = [b](double right, double left) {
    double angle = (right - left) / b;
    double radius = (right + left) / (2 * angle);
    return std::array<double, 3>{radius * (sin(1.2 + angle) - sin(1.2)),
                                 -radius * (cos(1.2 + angle) - cos(1.2)), angle};
  };
  const double step = 1e-7;
  const auto rp = increment(R + step, L), rm = increment(R - step, L);
  const auto lp = increment(R, L + step), lm = increment(R, L - step);
  double J[3][2];
  for (int i = 0; i < 3; ++i) {
    J[i][0] = (rp[i] - rm[i]) / (2 * step);
    J[i][1] = (lp[i] - lm[i]) / (2 * step);
  }
  double Q[2][2] = {{.02 * R, .3 * sqrt(.02 * R * .01 * L)}, {.3 * sqrt(.02 * R * .01 * L), .01 * L}};
  const auto p = robot.getPose().covariance;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double expected = 0;
      for (int a = 0; a < 2; ++a)
        for (int bidx = 0; bidx < 2; ++bidx)
          expected += J[i][a] * Q[a][bidx] * J[j][bidx];
      EXPECT_NEAR(p[i * 3 + j], expected, 2e-8);
    }
}

TEST(CreateOdometryCovariance, HeadingVarianceIsIndependentOfDurationAndPacketPartition)
{
  create::Create a(create::RobotModel::CREATE_2, false), b(create::RobotModel::CREATE_2, false);
  a.setOdometryCalibration(noiseCalibration(a));
  b.setOdometryCalibration(noiseCalibration(b));
  CreateTestAccess::sample(a, 0, 0, 1.0);
  CreateTestAccess::sample(b, 0, 0, 1.0);
  CreateTestAccess::sample(a, 20, 40, 1.1);
  for (int i = 1; i <= 4; ++i)
    CreateTestAccess::sample(b, 5 * i, 10 * i, 1.0 + .1 * i);
  EXPECT_NEAR(a.getPose().yaw, b.getPose().yaw, 1e-7);
  EXPECT_NEAR(a.getPose().covariance[8], b.getPose().covariance[8], 1e-8);
}
