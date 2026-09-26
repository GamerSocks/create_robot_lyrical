/**
Software License Agreement (BSD)

\file      test_create.cpp
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2018, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include "create/create.h"
#include "create/types.h"

#include "gtest/gtest.h"
#include <atomic>
#include <array>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <tuple>

TEST(CreateTest, ConstructorSingleParam)
{
  create::Create create_default;

  create::Create create_1(create::RobotModel::CREATE_1);

  create::Create create_2(create::RobotModel::CREATE_2);

  create::Create create_roomba_400(create::RobotModel::ROOMBA_400);
}

// TEST(CreateTest, ConstructorMultiParam)
// {
//   TODO(jacobperron): Document exception thrown and consider defining custom exception
//   create::Create create(std::string("/dev/ttyUSB0"), 11520);
// }

TEST(CreateTest, Connected)
{
  create::Create create;
  // Nothing to be connected to
  EXPECT_FALSE(create.connected());
}

TEST(CreateTest, Disconnect)
{
  create::Create create;
  // Even though not connected, this should not crash
  create.disconnect();
}

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
    EXPECT_EQ(state.sequence, 1u);
  }
}

TEST(CreateOdometry, MatchedWindowStartupIrregularIntervalsAndDroppedPackets)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(3);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 10, 10, 1.01);
  EXPECT_NEAR(robot.getVel().x, 1000 * METRES_PER_TICK, 1e-6);
  CreateTestAccess::sample(robot, 30, 30, 1.03);
  CreateTestAccess::sample(robot, 130, 130, 1.13); // missed samples retain cumulative ticks
  auto state = robot.getOdometryState();
  EXPECT_NEAR(state.window_duration, .13, 1e-9);
  EXPECT_NEAR(state.velocity.x, 1000 * METRES_PER_TICK, 1e-6);
  EXPECT_NEAR(state.left_velocity, state.velocity.x, 1e-7);
  CreateTestAccess::sample(robot, 130, 130, 1.15); // remove first interval from window
  state = robot.getOdometryState();
  EXPECT_NEAR(state.window_duration, .14, 1e-9);
  EXPECT_NEAR(state.velocity.x, 120 * METRES_PER_TICK / .14, 1e-6);
  EXPECT_NEAR(state.pose.x, 130 * METRES_PER_TICK, 1e-6);
  EXPECT_NEAR(state.window_left_travel, state.window_left_distance, 1e-9);
}

TEST(CreateOdometry, CurvesAndTurnsUseBodyWindowVelocities)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(1);
  const double axle = create::RobotModel::CREATE_2.getAxleLength();
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 90, 110, 1.1);
  auto state = robot.getOdometryState();
  EXPECT_NEAR(state.pose.x, 0.0, 1e-7);
  EXPECT_NEAR(state.pose.y, 0.0, 1e-7);
  EXPECT_NEAR(state.velocity.x, 0.0, 1e-7);
  EXPECT_NEAR(state.velocity.yaw, 20 * METRES_PER_TICK / axle / .1, 1e-6);
  EXPECT_NEAR(state.pose.yaw, 20 * METRES_PER_TICK / axle, 1e-7);
  const double oldYaw = state.pose.yaw;
  CreateTestAccess::sample(robot, 110, 150, 1.2);
  state = robot.getOdometryState();
  const double yawDelta = 20 * METRES_PER_TICK / axle;
  const double radius = 30 * METRES_PER_TICK / yawDelta;
  EXPECT_NEAR(state.pose.x, radius * (sin(oldYaw + yawDelta) - sin(oldYaw)), 1e-6);
  EXPECT_NEAR(state.pose.y, -radius * (cos(oldYaw + yawDelta) - cos(oldYaw)), 1e-6);
  EXPECT_NEAR(state.velocity.x, 30 * METRES_PER_TICK / .1, 1e-6);
  EXPECT_EQ(state.velocity.y, 0.0);
}

TEST(CreateOdometry, GapInvalidatesAndClearsWindowWithoutInventingMotion)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 110, 1.1);
  const auto before = robot.getOdometryState();
  CreateTestAccess::sample(robot, 1010, 1010, 2.2);
  auto state = robot.getOdometryState();
  EXPECT_FALSE(state.valid);
  EXPECT_EQ(state.sequence, before.sequence);
  EXPECT_EQ(state.sample_time, before.sample_time);
  EXPECT_EQ(state.discontinuities, 1u);
  EXPECT_FLOAT_EQ(state.pose.x, before.pose.x);
  CreateTestAccess::sample(robot, 1020, 1020, 2.3);
  state = robot.getOdometryState();
  EXPECT_TRUE(state.valid);
  EXPECT_NEAR(state.window_duration, .1, 1e-9);
  EXPECT_NEAR(state.pose.x, 20 * METRES_PER_TICK, 1e-7);
  EXPECT_NEAR(state.velocity.x, 10 * METRES_PER_TICK / .1, 1e-6);
}

TEST(CreateOdometry, ZeroIntervalPreservesCumulativeMovementAndHalfRangeIsRejected)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 5, 5, 1.0);
  CreateTestAccess::sample(robot, 10, 10, 1.1);
  EXPECT_NEAR(robot.getPose().x, 10 * METRES_PER_TICK, 1e-7);
  CreateTestAccess::sample(robot, 32778, 10, 1.2);
  EXPECT_FALSE(robot.getOdometryState().valid);
  EXPECT_EQ(robot.getOdometryState().discontinuities, 1u);
  EXPECT_NEAR(robot.getPose().x, 10 * METRES_PER_TICK, 1e-7);
}

TEST(CreateOdometry, OptionalSpeedLimitAndConfigurationValidation)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  EXPECT_THROW(robot.setDtHistoryLength(0), std::invalid_argument);
  EXPECT_THROW(robot.setOdometryLimits(0), std::invalid_argument);
  EXPECT_THROW(robot.setOdometryLimits(1, -1), std::invalid_argument);
  robot.setOdometryLimits(1.0, .6, .05);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 1000, 1000, 1.01);
  EXPECT_FALSE(robot.getOdometryState().valid);
  EXPECT_EQ(robot.getOdometryState().discontinuities, 1u);
  CreateTestAccess::sample(robot, 1010, 1010, 1.02);
  EXPECT_TRUE(robot.getOdometryState().valid);
  EXPECT_NEAR(robot.getPose().x, 10 * METRES_PER_TICK, 1e-7);
}

TEST(CreateOdometry, SnapshotIsCoherentDuringConcurrentUpdates)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (int i = 1; i <= 2000; ++i)
      CreateTestAccess::sample(robot, i, i, 1.0 + .01 * i);
    done = true;
  });
  do {
    const auto state = robot.getOdometryState();
    if (state.valid) {
      EXPECT_NEAR(state.pose.x, state.sequence * METRES_PER_TICK, 3e-5);
      EXPECT_DOUBLE_EQ(state.left_distance, state.right_distance);
      EXPECT_DOUBLE_EQ(state.left_distance, state.pose.x);
      EXPECT_NEAR(state.velocity.x, state.window_left_distance / state.window_duration, 1e-6);
      EXPECT_EQ(state.pose.covariance.size(), 9u);
      EXPECT_EQ(state.velocity.covariance.size(), 9u);
    }
    (void)robot.getPose();
    (void)robot.getVel();
    (void)robot.getMeasuredLeftWheelVel();
    (void)robot.getRightWheelDistance();
  } while (!done);
  writer.join();
  EXPECT_EQ(robot.getOdometryState().sequence, 2000u);
}

TEST(CreateOdometry, ReversingWindowRetainsAbsoluteTravelForUncertainty)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(2);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 110, 1.1);
  CreateTestAccess::sample(robot, 100, 100, 1.2);
  const auto state = robot.getOdometryState();
  EXPECT_NEAR(state.velocity.x, 0.0, 1e-7);
  EXPECT_NEAR(state.window_left_distance, 0.0, 1e-7);
  EXPECT_NEAR(state.window_left_travel, 20 * METRES_PER_TICK, 1e-7);
  EXPECT_NEAR(state.window_right_travel, 20 * METRES_PER_TICK, 1e-7);
}

namespace {
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

TEST(CreateOdometryCovariance, AnalyticBodyCovarianceUsesMatchingWindowAndCrossTerm)
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
  for (int index : {1, 3, 5, 7})
    EXPECT_EQ(state.velocity.covariance[index], 0.0f);
  EXPECT_NEAR(state.velocity.covariance[4], calibration.lateral_velocity_variance, 1e-12);
}

TEST(CreateOdometryCovariance, HeadingInvariantAndInverseSquaredTimeScaling)
{
  create::Create a(create::RobotModel::CREATE_2, false), b(create::RobotModel::CREATE_2, false);
  a.setOdometryCalibration(noiseCalibration(a));
  b.setOdometryCalibration(noiseCalibration(b));
  CreateTestAccess::yaw(b, 1.7f);
  CreateTestAccess::sample(a, 0, 0, 1.0);
  CreateTestAccess::sample(b, 0, 0, 1.0);
  CreateTestAccess::sample(a, 20, 30, 1.1);
  CreateTestAccess::sample(b, 20, 30, 1.2);
  const auto ca = a.getVel().covariance;
  const auto cb = b.getVel().covariance;
  for (int index : {0, 2, 6, 8})
    EXPECT_NEAR(ca[index], 4 * cb[index], 1e-7);
  EXPECT_GT(std::abs(a.getPose().y - b.getPose().y), .001);
}

TEST(CreateOdometryCovariance, EndpointOffsetIsOncePerWindowRegardlessOfPacketCount)
{
  create::Create a(create::RobotModel::CREATE_2, false), b(create::RobotModel::CREATE_2, false);
  a.setOdometryCalibration(noiseCalibration(a));
  b.setOdometryCalibration(noiseCalibration(b));
  CreateTestAccess::sample(a, 0, 0, 1.0);
  CreateTestAccess::sample(b, 0, 0, 1.0);
  CreateTestAccess::sample(a, 20, 40, 1.1);
  for (int i = 1; i <= 4; ++i)
    CreateTestAccess::sample(b, 5 * i, 10 * i, 1.0 + .025 * i);
  for (int i : {0, 2, 6, 8})
    EXPECT_NEAR(a.getVel().covariance[i], b.getVel().covariance[i], 1e-7);
}

TEST(CreateOdometryCovariance, VelocityWindowStopsGrowingButPoseUncertaintyAccumulates)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(2);
  robot.setOdometryCalibration(noiseCalibration(robot));
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 10, 10, 1.1);
  CreateTestAccess::sample(robot, 20, 20, 1.2);
  const auto before = robot.getOdometryState();
  for (int i = 3; i <= 8; ++i)
    CreateTestAccess::sample(robot, 10 * i, 10 * i, 1.0 + .1 * i);
  const auto after = robot.getOdometryState();
  for (int i : {0, 2, 6, 8})
    EXPECT_NEAR(after.velocity.covariance[i], before.velocity.covariance[i], 1e-7);
  EXPECT_GT(after.pose.covariance[0], before.pose.covariance[0]);
  EXPECT_GT(after.pose.covariance[8], before.pose.covariance[8]);
}

TEST(CreateOdometryCovariance, ReverseTravelDoesNotCancelUncertainty)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(2);
  auto calibration = noiseCalibration(robot);
  robot.setOdometryCalibration(calibration);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 120, 1.1);
  CreateTestAccess::sample(robot, 100, 100, 1.2);
  EXPECT_NEAR(robot.getVel().x, 0, 1e-8);
  const double qr = .02 * 40 * METRES_PER_TICK + 4e-6;
  const double ql = .01 * 20 * METRES_PER_TICK + 1e-6;
  EXPECT_NEAR(robot.getVel().covariance[0], (qr + ql) / .16, 1e-8);
}

TEST(CreateOdometryCovariance, CorrelationLimitsPreservePositiveSemidefiniteCovariance)
{
  for (double rho : {-1.0, -.4, 0.0, .7, 1.0}) {
    create::Create robot(create::RobotModel::CREATE_2, false);
    auto calibration = noiseCalibration(robot);
    calibration.wheel_error_correlation = rho;
    robot.setOdometryCalibration(calibration);
    CreateTestAccess::sample(robot, 100, 100, 1.0);
    CreateTestAccess::sample(robot, 90, 130, 1.1);
    const auto c = robot.getVel().covariance;
    EXPECT_GE(c[0], 0.0f);
    EXPECT_GE(c[8], 0.0f);
    EXPECT_GE(static_cast<double>(c[0]) * c[8] - static_cast<double>(c[2]) * c[6], -1e-8);
    EXPECT_EQ(c[2], c[6]);
  }
}

TEST(CreateOdometryCovariance, StationaryOffsetsAndExplicitFloors)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  auto calibration = noiseCalibration(robot);
  robot.setOdometryCalibration(calibration);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 100, 100, 1.1);
  EXPECT_NEAR(robot.getVel().covariance[0], 5e-6 / .04, 1e-11);
  EXPECT_NEAR(robot.getVel().covariance[8], 5e-6 / (calibration.axle_length * calibration.axle_length * .01),
              1e-9);

  create::Create floorRobot(create::RobotModel::CREATE_2, false);
  calibration.left_noise_q0 = calibration.right_noise_q0 = 0.0;
  calibration.vx_variance_floor = .01;
  calibration.yaw_rate_variance_floor = .02;
  floorRobot.setOdometryCalibration(calibration);
  CreateTestAccess::sample(floorRobot, 100, 100, 1.0);
  CreateTestAccess::sample(floorRobot, 100, 100, 1.1);
  EXPECT_FLOAT_EQ(floorRobot.getVel().covariance[0], .01f);
  EXPECT_FLOAT_EQ(floorRobot.getVel().covariance[8], .02f);
}

TEST(CreateOdometryCovariance, BiasScalesAndEffectiveTrackApplyToPoseAndVelocity)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  robot.setDtHistoryLength(1);
  auto calibration = noiseCalibration(robot);
  calibration.left_distance_scale = .5;
  calibration.right_distance_scale = .25;
  calibration.axle_length = .30;
  robot.setOdometryCalibration(calibration);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 120, 140, 1.1);
  EXPECT_NEAR(robot.getPose().x, 10 * METRES_PER_TICK, 1e-8);
  EXPECT_NEAR(robot.getPose().yaw, 0.0, 1e-8);
  EXPECT_NEAR(robot.getVel().x, 10 * METRES_PER_TICK / .1, 1e-7);
  CreateTestAccess::sample(robot, 100, 180, 1.2);
  EXPECT_NEAR(robot.getPose().yaw, 20 * METRES_PER_TICK / .30, 1e-7);
  EXPECT_NEAR(robot.getVel().yaw, 20 * METRES_PER_TICK / (.30 * .1), 1e-6);
  const double qr = .02 * 10 * METRES_PER_TICK + 4e-6;
  const double ql = .01 * 10 * METRES_PER_TICK + 1e-6;
  EXPECT_NEAR(robot.getVel().covariance[8], (qr + ql) / (.30 * .30 * .01), 1e-7);
}

TEST(CreateOdometryCovariance, ExactPoseJacobianMatchesNumericalDifferentiation)
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

TEST(CreateOdometryCovariance, ConfigurationRejectsInvalidAndMidstreamCalibration)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  const auto valid = noiseCalibration(robot);
  for (double invalid :
       {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    for (auto field :
         {&create::OdometryCalibration::left_noise_k, &create::OdometryCalibration::right_noise_q0,
          &create::OdometryCalibration::vx_variance_floor,
          &create::OdometryCalibration::lateral_velocity_variance}) {
      auto calibration = valid;
      calibration.*field = invalid;
      EXPECT_THROW(robot.setOdometryCalibration(calibration), std::invalid_argument);
    }
  }
  auto calibration = valid;
  calibration.axle_length = 0;
  EXPECT_THROW(robot.setOdometryCalibration(calibration), std::invalid_argument);
  calibration = valid;
  calibration.wheel_error_correlation = 1.01;
  EXPECT_THROW(robot.setOdometryCalibration(calibration), std::invalid_argument);
  robot.setOdometryCalibration(valid);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  EXPECT_THROW(robot.setOdometryCalibration(valid), std::logic_error);
}

TEST(CreateOdometryCovariance, YawOrientationIncludesInitialDiffusionAndOneEndpointOffset)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  auto c = noiseCalibration(robot);
  c.initial_yaw_variance = .03;
  c.wheel_error_correlation = .2;
  robot.setOdometryCalibration(c);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  CreateTestAccess::sample(robot, 110, 120, 1.1);
  const double qr = .02 * 20 * METRES_PER_TICK;
  const double ql = .01 * 10 * METRES_PER_TICK;
  const double diffusion = (qr + ql - .4 * std::sqrt(qr * ql)) / (c.axle_length * c.axle_length);
  const double endpoint =
      (c.right_noise_q0 + c.left_noise_q0 - .4 * std::sqrt(c.right_noise_q0 * c.left_noise_q0)) /
      (c.axle_length * c.axle_length);
  EXPECT_NEAR(robot.getPose().covariance[8], .03 + diffusion + endpoint, 1e-8);
  CreateTestAccess::sample(robot, 120, 140, 1.2);
  EXPECT_NEAR(robot.getPose().covariance[8], .03 + 2 * diffusion + endpoint, 1e-8);
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
  EXPECT_NEAR(a.getVel().covariance[8], 16 * b.getVel().covariance[8], 1e-7);
}

TEST(CreateOdometryCovariance, StationaryHeadingOffsetDoesNotAccumulateAndFloorIsConfigurable)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  auto c = noiseCalibration(robot);
  c.yaw_orientation_variance_floor = .001;
  robot.setOdometryCalibration(c);
  CreateTestAccess::sample(robot, 100, 100, 1.0);
  for (int i = 1; i <= 20; ++i) {
    CreateTestAccess::sample(robot, 100, 100, 1.0 + .1 * i);
    EXPECT_FLOAT_EQ(robot.getPose().covariance[8], .001f);
  }
  for (auto field : {&create::OdometryCalibration::initial_yaw_variance,
                     &create::OdometryCalibration::yaw_orientation_variance_floor,
                     &create::OdometryCalibration::yaw_discontinuity_variance}) {
    create::Create other(create::RobotModel::CREATE_2, false);
    auto invalid = other.getOdometryCalibration();
    invalid.*field = -1;
    EXPECT_THROW(other.setOdometryCalibration(invalid), std::invalid_argument);
  }
}

TEST(CreateOdometryCovariance, LostMotionInflatesHeadingUncertaintyAcrossRecovery)
{
  create::Create robot(create::RobotModel::CREATE_2, false);
  auto c = noiseCalibration(robot);
  c.yaw_discontinuity_variance = .7;
  robot.setOdometryCalibration(c);
  CreateTestAccess::sample(robot, 0, 0, 1.0);
  CreateTestAccess::sample(robot, 10, 10, 1.1);
  const auto before = robot.getOdometryState();
  CreateTestAccess::sample(robot, 500, 800, 2.2);
  const auto invalid = robot.getOdometryState();
  EXPECT_FALSE(invalid.valid);
  EXPECT_NEAR(invalid.pose.covariance[8], before.pose.covariance[8] + .7, 1e-7);
  CreateTestAccess::sample(robot, 500, 800, 2.3);
  const auto recovered = robot.getOdometryState();
  EXPECT_TRUE(recovered.valid);
  EXPECT_FLOAT_EQ(recovered.pose.yaw, before.pose.yaw);
  EXPECT_NEAR(recovered.pose.covariance[8], invalid.pose.covariance[8], 1e-7);
}
