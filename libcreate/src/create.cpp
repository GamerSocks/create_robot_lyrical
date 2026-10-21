#include <algorithm>
#include <iostream>
#include <cmath>
#include <ctime>
#include <memory>
#include <assert.h>
#include <stdexcept>

#include "create/create.h"

#define GET_DATA(id) (data->getPacket(id)->getData())
#define BOUND_CONST(val,min,max) (val<min?min:(val>max?max:val))
#define BOUND(val,min,max) (val = BOUND_CONST(val,min,max))

namespace create {

  namespace ublas = boost::numeric::ublas;

  void Create::init(bool install_signal_handler) {
    mainMotorPower = 0;
    sideMotorPower = 0;
    vacuumMotorPower = 0;
    debrisLED = 0;
    spotLED = 0;
    dockLED = 0;
    checkLED = 0;
    powerLED = 0;
    powerLEDIntensity = 0;
    prevTicksLeft = 0;
    prevTicksRight = 0;
    totalLeftDist = 0.0;
    totalRightDist = 0.0;
    firstOnData = true;
    mode = MODE_OFF;
    pose.x = 0;
    pose.y = 0;
    pose.yaw = 0;
    pose.covariance = std::vector<float>(9, 0.0);
    vel.x = 0;
    vel.y = 0;
    vel.yaw = 0;
    vel.covariance = std::vector<float>(9, 0.0);
    poseCovar = Matrix(3, 3, 0.0);
    requestedLeftVel = 0;
    requestedRightVel = 0;
    measuredLeftVel = 0;
    measuredRightVel = 0;
    dtHistoryLength = 5;
    odometryMaxGap = 1.0;
    maxEncoderWheelSpeed = 0.0;
    encoderTimingTolerance = 0.05;
    odometryCalibration.axle_length = model.getAxleLength();
    if (model.getVersion() >= V_3) {
      const double tick = model.getWheelDiameter() * util::PI / util::V_3_TICKS_PER_REV;
      // Moving-phase endpoint quantization approximation; not calibrated slip noise.
      odometryCalibration.left_noise_q0 = tick * tick / 6.0;
      odometryCalibration.right_noise_q0 = tick * tick / 6.0;
    }
    odometryState.pose = pose;
    odometryState.velocity = vel;
    modeReportWorkaround = false;
    data = std::shared_ptr<Data>(new Data(model.getVersion()));
    if (model.getVersion() == V_1) {
      serial = std::make_shared<SerialQuery>(data, install_signal_handler);
    } else {
      serial = std::make_shared<SerialStream>(
        data, create::util::STREAM_HEADER, install_signal_handler);
    }
  }

  Create::Create(RobotModel m, bool install_signal_handler) : model(m) {
    init(install_signal_handler);
  }

  Create::Create(const std::string& dev, const int& baud, RobotModel m, bool install_signal_handler)
    : model(m)
  {
    init(install_signal_handler);
    serial->connect(dev, baud);
  }

  Create::~Create() {
    disconnect();
  }

  Create::Matrix Create::addMatrices(const Matrix &A, const Matrix &B) const {
    size_t rows = A.size1();
    size_t cols = A.size2();

    assert(rows == B.size1());
    assert(cols == B.size2());

    Matrix C(rows, cols);
    for (size_t i = 0u; i < rows; i++) {
      for (size_t j = 0u; j < cols; j++) {
        const float a = A(i, j);
        const float b = B(i, j);
        if (util::willFloatOverflow(a, b)) {
          // If overflow, set to float min or max depending on direction of overflow
          C(i, j) = (a < 0.0) ? std::numeric_limits<float>::min() : std::numeric_limits<float>::max();
        }
        else {
          C(i, j) = a + b;
        }
      }
    }
    return C;
  }

  void Create::updatePoseCovariance() {
    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        pose.covariance[3 * row + col] = poseCovar(row, col);
      }
    }
    const auto& c = odometryCalibration;
    // Endpoint error of heading since the initial encoder baseline, added once
    // to the published marginal. Never feed it back into the diffusion state.
    const double endpointCross = c.wheel_error_correlation *
      std::sqrt(c.right_noise_q0) * std::sqrt(c.left_noise_q0);
    const double endpointYaw = std::max(0.0,
      (c.right_noise_q0 + c.left_noise_q0 - 2.0 * endpointCross) / (c.axle_length * c.axle_length));
    pose.covariance[8] = std::max(c.yaw_orientation_variance_floor,
      static_cast<double>(poseCovar(2, 2)) + endpointYaw);
  }

  void Create::rebaseline(std::chrono::steady_clock::time_point curTime,
                         bool discontinuity) {
    if (model.getVersion() >= V_3) {
      prevTicksLeft = GET_DATA(ID_LEFT_ENC);
      prevTicksRight = GET_DATA(ID_RIGHT_ENC);
    }
    prevOnDataTime = curTime;
    firstOnData = false;
    velocityHistory.clear();
    measuredLeftVel = measuredRightVel = 0.0f;
    vel.x = vel.y = vel.yaw = 0.0f;
    odometryState.valid = false;
    odometryState.window_duration = 0.0;
    odometryState.window_left_distance = odometryState.window_right_distance = 0.0;
    odometryState.window_left_travel = odometryState.window_right_travel = 0.0;
    if (discontinuity) {
      ++odometryState.discontinuities;
      // Lost encoder motion also loses heading: do not retain false confidence
      // in an unchanged yaw when the next valid sample resumes publication.
      poseCovar(2, 2) += odometryCalibration.yaw_discontinuity_variance;
      updatePoseCovariance();
      odometryState.pose = pose;
    }
  }

  void Create::onData() {
    // Timestamp reception before waiting for any reader of the odometry state.
    onDataAt(std::chrono::steady_clock::now());
  }

  void Create::onDataAt(std::chrono::steady_clock::time_point curTime) {
    std::lock_guard<std::mutex> lock(odometryMutex);
    if (firstOnData) {
      rebaseline(curTime, false);
      return;
    }
    const double dt = std::chrono::duration<double>(curTime - prevOnDataTime).count();
    // Cumulative encoders preserve this movement for the next timed sample.
    if (dt == 0.0 && model.getVersion() >= V_3) return;
    if (dt <= 0.0 || dt > odometryMaxGap) {
      rebaseline(curTime, true);
      return;
    }
    float deltaDist = 0.0f;
    float deltaX = 0.0f;
    float deltaY = 0.0f;
    float deltaYaw = 0.0f;
    float leftWheelDist = 0.0f;
    float rightWheelDist = 0.0f;
    float wheelDistDiff = 0.0f;

    // Protocol versions 1 and 2 use distance and angle fields for odometry
    int16_t angleField = 0;
    if (model.getVersion() <= V_2) {
      // This is a standards compliant way of doing unsigned to signed conversion
      uint16_t distanceRaw = GET_DATA(ID_DISTANCE);
      int16_t distance;
      std::memcpy(&distance, &distanceRaw, sizeof(distance));
      deltaDist = distance / 1000.0; // mm -> m

      // Angle is processed differently in versions 1 and 2
      uint16_t angleRaw = GET_DATA(ID_ANGLE);
      std::memcpy(&angleField, &angleRaw, sizeof(angleField));
    }

    if (model.getVersion() == V_1) {
      wheelDistDiff = 2.0 * angleField / 1000.0;
      leftWheelDist = deltaDist - (wheelDistDiff / 2.0);
      rightWheelDist = deltaDist + (wheelDistDiff / 2.0);
      deltaYaw = wheelDistDiff / model.getAxleLength();
    } else if (model.getVersion() == V_2) {
      /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
       * Certain older Creates have major problems with odometry                   *
       * http://answers.ros.org/question/31935/createroomba-odometry/              *
       *                                                                           *
       * All Creates have an issue with rounding of the angle field, which causes  *
       * major errors to accumulate in the odometry yaw.                           *
       * http://wiki.tekkotsu.org/index.php/Create_Odometry_Bug                    *
       * https://github.com/AutonomyLab/create_autonomy/issues/28                  *
       *                                                                           *
       * TODO: Consider using velocity command as substitute for pose estimation   *
       * to mitigate both of these problems.                                       *
       * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
      deltaYaw = angleField * (util::PI / 180.0); // D2R
      wheelDistDiff = model.getAxleLength() * deltaYaw;
      leftWheelDist = deltaDist - (wheelDistDiff / 2.0);
      rightWheelDist = deltaDist + (wheelDistDiff / 2.0);
    } else if (model.getVersion() >= V_3) {
      // Get cumulative ticks (wraps around at 65535)
      uint16_t totalTicksLeft = GET_DATA(ID_LEFT_ENC);
      uint16_t totalTicksRight = GET_DATA(ID_RIGHT_ENC);
      // Signed shortest displacement modulo 2^16. An interval must contain
      // fewer than half a revolution of the counter, not of the wheel.
      const auto tickDelta = [](uint16_t current, uint16_t previous) {
        int32_t delta = static_cast<int32_t>(current) - static_cast<int32_t>(previous);
        if (delta > 32767) delta -= 65536;
        else if (delta < -32768) delta += 65536;
        return delta;
      };
      const int32_t ticksLeft = tickDelta(totalTicksLeft, prevTicksLeft);
      const int32_t ticksRight = tickDelta(totalTicksRight, prevTicksRight);
      if (ticksLeft == -32768 || ticksRight == -32768) {
        // Exactly half the counter range has two equally plausible directions.
        rebaseline(curTime, true);
        return;
      }
      prevTicksLeft = totalTicksLeft;
      prevTicksRight = totalTicksRight;

      // Compute distance travelled by each wheel
      leftWheelDist = (ticksLeft / util::V_3_TICKS_PER_REV)
          * model.getWheelDiameter() * util::PI;
      rightWheelDist = (ticksRight / util::V_3_TICKS_PER_REV)
          * model.getWheelDiameter() * util::PI;
      deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

      wheelDistDiff = rightWheelDist - leftWheelDist;
      deltaYaw = wheelDistDiff / model.getAxleLength();
    }

    const auto& calibration = odometryCalibration;
    const double axle = calibration.axle_length;
    leftWheelDist *= calibration.left_distance_scale;
    rightWheelDist *= calibration.right_distance_scale;
    deltaDist = (rightWheelDist + leftWheelDist) / 2.0;
    wheelDistDiff = rightWheelDist - leftWheelDist;
    deltaYaw = wheelDistDiff / axle;

    const double metresPerTick = model.getWheelDiameter() * util::PI / util::V_3_TICKS_PER_REV;
    if (model.getVersion() >= V_3 && maxEncoderWheelSpeed > 0.0) {
      const double tickLeft = metresPerTick * calibration.left_distance_scale;
      const double tickRight = metresPerTick * calibration.right_distance_scale;
      const double boundLeft = maxEncoderWheelSpeed * (dt + encoderTimingTolerance) + tickLeft;
      const double boundRight = maxEncoderWheelSpeed * (dt + encoderTimingTolerance) + tickRight;
      if (boundLeft >= 32768.0 * tickLeft || boundRight >= 32768.0 * tickRight ||
          std::abs(leftWheelDist) > boundLeft || std::abs(rightWheelDist) > boundRight) {
        rebaseline(curTime, true);
        return;
      }
    }

    velocityHistory.push_back({dt, leftWheelDist, rightWheelDist});
    while (velocityHistory.size() > dtHistoryLength) velocityHistory.pop_front();
    double windowTime = 0.0, windowLeft = 0.0, windowRight = 0.0;
    double leftTravel = 0.0, rightTravel = 0.0;
    for (const auto& interval : velocityHistory) {
      windowTime += interval.dt;
      windowLeft += interval.left;
      windowRight += interval.right;
      leftTravel += std::abs(interval.left);
      rightTravel += std::abs(interval.right);
    }
    measuredLeftVel = windowLeft / windowTime;
    measuredRightVel = windowRight / windowTime;

    // Exact constant-curvature integration, stable at zero curvature.
    const double halfYaw = deltaYaw / 2.0;
    const double halfYaw2 = halfYaw * halfYaw;
    const double sinc = std::abs(halfYaw) < 1e-4 ?
      1.0 - halfYaw2 / 6.0 + halfYaw2 * halfYaw2 / 120.0 : std::sin(halfYaw) / halfYaw;
    // Derivative of sinc(deltaYaw/2) with respect to deltaYaw.
    const double sincDerivative = std::abs(halfYaw) < 1e-4 ?
      -halfYaw / 6.0 + halfYaw * halfYaw2 / 60.0 :
      (halfYaw * std::cos(halfYaw) - std::sin(halfYaw)) / (2.0 * halfYaw2);
    const double cosMid = std::cos(pose.yaw + halfYaw);
    const double sinMid = std::sin(pose.yaw + halfYaw);
    deltaX = deltaDist * sinc * cosMid;
    deltaY = deltaDist * sinc * sinMid;

    totalLeftDist += leftWheelDist;
    totalRightDist += rightWheelDist;

    vel.x = (windowRight + windowLeft) / (2.0 * windowTime);
    vel.y = 0.0;
    vel.yaw = (windowRight - windowLeft) / (axle * windowTime);

    // Body-frame velocity covariance of the same complete measurement window.
    // q0 is added once, never independently to each encoder difference.
    const double qRight = calibration.right_noise_k * rightTravel + calibration.right_noise_q0;
    const double qLeft = calibration.left_noise_k * leftTravel + calibration.left_noise_q0;
    const double cross = calibration.wheel_error_correlation * std::sqrt(qRight) * std::sqrt(qLeft);
    const double timeSquared = windowTime * windowTime;
    std::fill(vel.covariance.begin(), vel.covariance.end(), 0.0f);
    vel.covariance[0] = std::max(calibration.vx_variance_floor,
      (qRight + qLeft + 2.0 * cross) / (4.0 * timeSquared));
    vel.covariance[4] = calibration.lateral_velocity_variance;
    vel.covariance[8] = std::max(calibration.yaw_rate_variance_floor,
      (qRight + qLeft - 2.0 * cross) / (axle * axle * timeSquared));
    vel.covariance[2] = vel.covariance[6] = (qRight - qLeft) / (2.0 * axle * timeSquared);

    // Accumulated pose diffusion uses this interval, not the rolling window.
    // q0 describes window endpoint uncertainty; treating it as fresh independent
    // pose noise every packet would double-count shared encoder endpoints.
    Matrix incrementNoise(2, 2);
    incrementNoise(0, 0) = calibration.right_noise_k * std::abs(rightWheelDist);
    incrementNoise(1, 1) = calibration.left_noise_k * std::abs(leftWheelDist);
    incrementNoise(0, 1) = incrementNoise(1, 0) = calibration.wheel_error_correlation *
      std::sqrt(incrementNoise(0, 0)) * std::sqrt(incrementNoise(1, 1));

    // Jacobians of the exact integration above (wheel order: right, left).
    const double dxDTheta = deltaDist * (sincDerivative * cosMid - 0.5 * sinc * sinMid);
    const double dyDTheta = deltaDist * (sincDerivative * sinMid + 0.5 * sinc * cosMid);
    Matrix Finc(3, 2);
    Finc(0, 0) = 0.5 * sinc * cosMid + dxDTheta / axle;
    Finc(0, 1) = 0.5 * sinc * cosMid - dxDTheta / axle;
    Finc(1, 0) = 0.5 * sinc * sinMid + dyDTheta / axle;
    Finc(1, 1) = 0.5 * sinc * sinMid - dyDTheta / axle;
    Finc(2, 0) = 1.0 / axle;
    Finc(2, 1) = -1.0 / axle;
    Matrix FincT = ublas::trans(Finc);
    Matrix incrementCovar = ublas::prod(incrementNoise, FincT);
    incrementCovar = ublas::prod(Finc, incrementCovar);

    Matrix Fp(3, 3, 0.0);
    Fp(0, 0) = Fp(1, 1) = Fp(2, 2) = 1.0;
    Fp(0, 2) = -deltaY;
    Fp(1, 2) = deltaX;
    Matrix FpT = ublas::trans(Fp);
    Matrix poseCovarTmp = ublas::prod(poseCovar, FpT);
    poseCovarTmp = ublas::prod(Fp, poseCovarTmp);
    poseCovar = addMatrices(poseCovarTmp, incrementCovar);

    updatePoseCovariance();

    // Update pose
    pose.x += deltaX;
    pose.y += deltaY;
    pose.yaw = util::normalizeAngle(pose.yaw + deltaYaw);

    prevOnDataTime = curTime;
    odometryState.pose = pose;
    odometryState.velocity = vel;
    odometryState.sample_time = curTime;
    ++odometryState.sequence;
    odometryState.valid = true;
    odometryState.left_distance = totalLeftDist;
    odometryState.right_distance = totalRightDist;
    odometryState.left_velocity = measuredLeftVel;
    odometryState.right_velocity = measuredRightVel;
    odometryState.window_duration = windowTime;
    odometryState.window_left_distance = windowLeft;
    odometryState.window_right_distance = windowRight;
    odometryState.window_left_travel = leftTravel;
    odometryState.window_right_travel = rightTravel;

    // Make user registered callbacks, if any
    // TODO
  }

  bool Create::connect(const std::string& port, const int& baud) {
    bool timeout = false;
    time_t start, now;
    float maxWait = 30; // seconds
    float retryInterval = 5; //seconds
    time(&start);
    while (!serial->connect(port, baud, std::bind(&Create::onData, this)) && !timeout) {
      time(&now);
      if (difftime(now, start) > maxWait) {
        timeout = true;
        CERR("[create::Create] ", "failed to connect over serial: timeout");
      }
      else {
        usleep(retryInterval * 1000000);
        COUT("[create::Create] ", "retrying to establish serial connection...");
      }
    }

    return !timeout;
  }

  void Create::disconnect() {
    serial->disconnect();
    std::lock_guard<std::mutex> lock(odometryMutex);
    firstOnData = true;
    odometryState.valid = false;
    velocityHistory.clear();
  }

  //void Create::reset() {
  //  serial->sendOpcode(OC_RESET);
  //  serial->reset(); // better
    // TODO : Should we request reading packets again?
  //}

  bool Create::setMode(const CreateMode& mode) {
    if (model.getVersion() == V_1){
      // Switch to safe mode (required for compatibility with V_1)
      if (!(serial->sendOpcode(OC_START) && serial->sendOpcode(OC_CONTROL))) return false;
    }
    bool ret = false;
    switch (mode) {
      case MODE_OFF:
        if (model.getVersion() == V_2) {
          CERR("[create::Create] ", "protocol version 2 does not support turning robot off");
          ret = false;
        } else {
          ret = serial->sendOpcode(OC_POWER);
        }
        break;
      case MODE_PASSIVE:
        ret = serial->sendOpcode(OC_START);
        break;
      case MODE_SAFE:
        if (model.getVersion() > V_1) {
          ret = serial->sendOpcode(OC_SAFE);
        }
        break;
      case MODE_FULL:
        ret = serial->sendOpcode(OC_FULL);
        break;
      default:
        CERR("[create::Create] ", "cannot set robot to mode '" << mode << "'");
        ret = false;
    }
    if (ret) {
      this->mode = mode;
    }
    return ret;
  }

  bool Create::clean(const CleanMode& mode) {
    return serial->sendOpcode((Opcode) mode);
  }

  bool Create::dock() const {
    return serial->sendOpcode(OC_DOCK);
  }

  bool Create::setDate(const DayOfWeek& day, const uint8_t& hour, const uint8_t& min) const {
    if (day < 0 || day > 6 ||
        hour > 23 ||
        min > 59)
      return false;

    uint8_t cmd[4] = { OC_DATE, static_cast<uint8_t>(day), hour, min };
    return serial->send(cmd, 4);
  }

  bool Create::driveRadius(const float& vel, const float& radius) {
    // Bound velocity
    float boundedVel = BOUND_CONST(vel, -model.getMaxVelocity(), model.getMaxVelocity());

    // Expects each parameter as two bytes each and in millimeters
    int16_t vel_mm = roundf(boundedVel * 1000);
    int16_t radius_mm = roundf(radius * 1000);

    // Bound radius if not a special case
    if (radius_mm != -32768 && radius_mm != 32767 &&
        radius_mm != -1 && radius_mm != 1) {
      BOUND(radius_mm, -util::MAX_RADIUS * 1000, util::MAX_RADIUS * 1000);
    }

    uint8_t cmd[5] = { OC_DRIVE,
                       static_cast<uint8_t>(vel_mm >> 8),
                       static_cast<uint8_t>(vel_mm & 0xff),
                       static_cast<uint8_t>(radius_mm >> 8),
                       static_cast<uint8_t>(radius_mm & 0xff)
                     };

    return serial->send(cmd, 5);
  }

  bool Create::driveWheels(const float& leftVel, const float& rightVel) {
    const float boundedLeftVel = BOUND_CONST(leftVel, -model.getMaxVelocity(), model.getMaxVelocity());
    const float boundedRightVel = BOUND_CONST(rightVel, -model.getMaxVelocity(), model.getMaxVelocity());
    {
      std::lock_guard<std::mutex> lock(odometryMutex);
      requestedLeftVel = boundedLeftVel;
      requestedRightVel = boundedRightVel;
    }
    if (model.getVersion() > V_1) {
      int16_t leftCmd = roundf(boundedLeftVel * 1000);
      int16_t rightCmd = roundf(boundedRightVel * 1000);

      uint8_t cmd[5] = { OC_DRIVE_DIRECT,
                         static_cast<uint8_t>(rightCmd >> 8),
                         static_cast<uint8_t>(rightCmd & 0xff),
                         static_cast<uint8_t>(leftCmd >> 8),
                         static_cast<uint8_t>(leftCmd & 0xff)
                       };
      return serial->send(cmd, 5);
    } else {
      float radius;
      // Prevent divide by zero when driving straight
      if (boundedLeftVel != boundedRightVel) {
        radius = -((model.getAxleLength() / 2.0) * (boundedLeftVel + boundedRightVel)) /
          (boundedLeftVel - boundedRightVel);
      } else {
        radius = util::STRAIGHT_RADIUS;
      }

      float vel;
      // Fix signs for spin in place
      if (boundedLeftVel == -boundedRightVel || std::abs(roundf(radius * 1000)) <= 1) {
        radius = util::IN_PLACE_RADIUS;
        vel = boundedRightVel;
      } else {
        vel = (std::abs(boundedLeftVel) + std::abs(boundedRightVel)) / 2.0 * ((boundedLeftVel + boundedRightVel) > 0 ? 1.0 : -1.0);
      }

      // Radius turns have a maximum radius of 2.0 meters
      // When the radius is > 2 but <= 10, use a 2 meter radius
      // When it is > 10, drive straight
      // TODO: alternate between these 2 meter radius and straight to
      // fake a larger radius
      if (radius > 10.0) {
        radius = util::STRAIGHT_RADIUS;
      }

      return driveRadius(vel, radius);
    }
  }

  bool Create::driveWheelsPwm(const float& leftWheel, const float& rightWheel)
  {
    static const int16_t PWM_COUNTS = 255;

    if (leftWheel < -1.0 || leftWheel > 1.0 ||
        rightWheel < -1.0 || rightWheel > 1.0)
      return false;

    int16_t leftPwm = roundf(leftWheel * PWM_COUNTS);
    int16_t rightPwm = roundf(rightWheel * PWM_COUNTS);

    uint8_t cmd[5] = { OC_DRIVE_PWM,
                       static_cast<uint8_t>(rightPwm >> 8),
                       static_cast<uint8_t>(rightPwm & 0xff),
                       static_cast<uint8_t>(leftPwm >> 8),
                       static_cast<uint8_t>(leftPwm & 0xff)
                     };

    return serial->send(cmd, 5);
  }

  bool Create::drive(const float& xVel, const float& angularVel) {
    // Compute left and right wheel velocities
    float leftVel = xVel - ((model.getAxleLength() / 2.0) * angularVel);
    float rightVel = xVel + ((model.getAxleLength() / 2.0) * angularVel);
    return driveWheels(leftVel, rightVel);
  }

  bool Create::setAllMotors(const float& main, const float& side, const float& vacuum) {
    if (main < -1.0 || main > 1.0 ||
        side < -1.0 || side > 1.0 ||
        vacuum < -1.0 || vacuum > 1.0)
      return false;

    mainMotorPower = roundf(main * 127);
    sideMotorPower = roundf(side * 127);
    vacuumMotorPower = roundf(vacuum * 127);

    if (model.getVersion() == V_1) {
        uint8_t cmd[2] = { OC_MOTORS,
                           static_cast<uint8_t>((side != 0.0 ? 1 : 0) |
                                     (vacuum != 0.0 ? 2 : 0) |
                                     (main != 0.0 ? 4 : 0))
                         };
        return serial->send(cmd, 2);
    }

    uint8_t cmd[4] = { OC_MOTORS_PWM,
                       mainMotorPower,
                       sideMotorPower,
                       vacuumMotorPower
                     };

    return serial->send(cmd, 4);
  }

  bool Create::setMainMotor(const float& main) {
    return setAllMotors(main, static_cast<float>(sideMotorPower) / 127.0, static_cast<float>(vacuumMotorPower) / 127.0);
  }

  bool Create::setSideMotor(const float& side) {
    return setAllMotors(static_cast<float>(mainMotorPower) / 127.0, side, static_cast<float>(vacuumMotorPower) / 127.0);
  }

  bool Create::setVacuumMotor(const float& vacuum) {
    return setAllMotors(static_cast<float>(mainMotorPower) / 127.0, static_cast<float>(sideMotorPower) / 127.0, vacuum);
  }

  bool Create::updateLEDs() {
    uint8_t LEDByte = debrisLED + spotLED + dockLED + checkLED;
    uint8_t cmd[4] = { OC_LEDS,
                       LEDByte,
                       powerLED,
                       powerLEDIntensity
                     };

    return serial->send(cmd, 4);
  }

  bool Create::enableDebrisLED(const bool& enable) {
    if (enable)
      debrisLED = LED_DEBRIS;
    else
      debrisLED = 0;
    return updateLEDs();
  }

  bool Create::enableSpotLED(const bool& enable) {
    if (enable)
      spotLED = LED_SPOT;
    else
      spotLED = 0;
    return updateLEDs();
  }

  bool Create::enableDockLED(const bool& enable) {
    if (enable)
      dockLED = LED_DOCK;
    else
      dockLED = 0;
    return updateLEDs();
  }

  bool Create::enableCheckRobotLED(const bool& enable) {
    if (enable)
      checkLED = LED_CHECK;
    else
      checkLED = 0;
    return updateLEDs();
  }

  bool Create::setPowerLED(const uint8_t& power, const uint8_t& intensity) {
    powerLED = power;
    powerLEDIntensity = intensity;
    return updateLEDs();
  }

  //void Create::setDigits(uint8_t digit1, uint8_t digit2,
  //                       uint8_t digit3, uint8_t digit4) {
  //}

  bool Create::setDigitsASCII(const uint8_t& digit1, const uint8_t& digit2,
                              const uint8_t& digit3, const uint8_t& digit4) const {
    if (digit1 < 32 || digit1 > 126 ||
        digit2 < 32 || digit2 > 126 ||
        digit3 < 32 || digit3 > 126 ||
        digit4 < 32 || digit4 > 126)
      return false;

    uint8_t cmd[5] = { OC_DIGIT_LEDS_ASCII,
                        digit1,
                        digit2,
                        digit3,
                        digit4
                      };

    return serial->send(cmd, 5);
  }

  bool Create::defineSong(const uint8_t& songNumber,
                          const uint8_t& songLength,
                          const uint8_t* notes,
                          const float* durations) const {
    int i, j;
    uint8_t duration;
    std::vector<uint8_t> cmd(2 * songLength + 3);
    cmd[0] = OC_SONG;
    cmd[1] = songNumber;
    cmd[2] = songLength;
    j = 0;
    for (i = 3; i < 2 * songLength + 3; i = i + 2) {
      if (durations[j] < 0 || durations[j] >= 4)
        return false;
      duration = durations[j] * 64;
      cmd[i] = notes[j];
      cmd[i + 1] = duration;
      j++;
    }

    return serial->send(cmd.data(), cmd.size());
  }

  bool Create::playSong(const uint8_t& songNumber) const {
    if (songNumber > 4)
      return false;
    uint8_t cmd[2] = { OC_PLAY, songNumber };
    return serial->send(cmd, 2);
  }

  void Create::setDtHistoryLength(const uint8_t& dtHistoryLength) {
    if (dtHistoryLength == 0) throw std::invalid_argument("velocity window must contain at least one interval");
    std::lock_guard<std::mutex> lock(odometryMutex);
    this->dtHistoryLength = dtHistoryLength;
    velocityHistory.clear();
  }

  void Create::setOdometryLimits(double max_gap, double max_wheel_speed,
                                double timing_tolerance) {
    if (!std::isfinite(max_gap) || max_gap <= 0.0 ||
        !std::isfinite(max_wheel_speed) || max_wheel_speed < 0.0 ||
        !std::isfinite(timing_tolerance) || timing_tolerance < 0.0) {
      throw std::invalid_argument("invalid odometry limits");
    }
    std::lock_guard<std::mutex> lock(odometryMutex);
    odometryMaxGap = max_gap;
    maxEncoderWheelSpeed = max_wheel_speed;
    encoderTimingTolerance = timing_tolerance;
  }

  OdometryState Create::getOdometryState() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return odometryState;
  }

  void Create::setOdometryCalibration(const OdometryCalibration& calibration) {
    for (double value : {calibration.left_distance_scale, calibration.right_distance_scale,
                         calibration.axle_length}) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("odometry geometry must be finite and positive");
    }
    for (double value : {calibration.left_noise_k, calibration.right_noise_k,
                         calibration.left_noise_q0, calibration.right_noise_q0,
                         calibration.vx_variance_floor, calibration.yaw_rate_variance_floor,
                         calibration.lateral_velocity_variance, calibration.initial_yaw_variance,
                         calibration.yaw_orientation_variance_floor, calibration.yaw_discontinuity_variance}) {
      if (!std::isfinite(value) || value < 0.0)
        throw std::invalid_argument("odometry noise variances must be finite and nonnegative");
    }
    if (!std::isfinite(calibration.wheel_error_correlation) ||
        std::abs(calibration.wheel_error_correlation) > 1.0)
      throw std::invalid_argument("wheel error correlation must be in [-1, 1]");
    std::lock_guard<std::mutex> lock(odometryMutex);
    if (!firstOnData || odometryState.sequence != 0)
      throw std::logic_error("configure odometry calibration before receiving sensor data");
    odometryCalibration = calibration;
    poseCovar(2, 2) = calibration.initial_yaw_variance;
    updatePoseCovariance();
    odometryState.pose = pose;
  }

  OdometryCalibration Create::getOdometryCalibration() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return odometryCalibration;
  }

  bool Create::isWheeldrop() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x0C) != 0;
    }
    else {
      CERR("[create::Create] ", "Wheeldrop sensor not supported!");
      return false;
    }
  }

  bool Create::isLeftWheeldrop() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x08) != 0;
    }
    else {
      CERR("[create::Create] ", "Wheeldrop sensor not supported!");
      return false;
    }
  }

  bool Create::isRightWheeldrop() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Wheeldrop sensor not supported!");
      return false;
    }
  }

  bool Create::isLeftBumper() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x02) != 0;
    }
    else {
      CERR("[create::Create] ", "Left bumper not supported!");
      return false;
    }
  }

  bool Create::isRightBumper() const {
    if (data->isValidPacketID(ID_BUMP_WHEELDROP)) {
      return (GET_DATA(ID_BUMP_WHEELDROP) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Right bumper not supported!");
      return false;
    }
  }

  bool Create::isWall() const {
    if (data->isValidPacketID(ID_WALL)) {
      return GET_DATA(ID_WALL) == 1;
    }
    else {
      CERR("[create::Create] ", "Wall sensor not supported!");
      return false;
    }
  }

  bool Create::isCliff() const {
    if (data->isValidPacketID(ID_CLIFF_LEFT) &&
        data->isValidPacketID(ID_CLIFF_FRONT_LEFT) &&
        data->isValidPacketID(ID_CLIFF_FRONT_RIGHT) &&
        data->isValidPacketID(ID_CLIFF_RIGHT)) {
      return GET_DATA(ID_CLIFF_LEFT) == 1 ||
             GET_DATA(ID_CLIFF_FRONT_LEFT) == 1 ||
             GET_DATA(ID_CLIFF_FRONT_RIGHT) == 1 ||
             GET_DATA(ID_CLIFF_RIGHT) == 1;
    }
    else {
      CERR("[create::Create] ", "Cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffLeft() const {
    if (data->isValidPacketID(ID_CLIFF_LEFT)) {
      return GET_DATA(ID_CLIFF_LEFT) == 1;
    }
    else {
      CERR("[create::Create] ", "Left cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffFrontLeft() const {
    if (data->isValidPacketID(ID_CLIFF_FRONT_LEFT)) {
      return GET_DATA(ID_CLIFF_FRONT_LEFT) == 1;
    }
    else {
      CERR("[create::Create] ", "Front left cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffRight() const {
    if (data->isValidPacketID(ID_CLIFF_RIGHT)) {
      return GET_DATA(ID_CLIFF_RIGHT) == 1;
    }
    else {
      CERR("[create::Create] ", "Rightt cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isCliffFrontRight() const {
    if (data->isValidPacketID(ID_CLIFF_FRONT_RIGHT)) {
      return GET_DATA(ID_CLIFF_FRONT_RIGHT) == 1;
    }
    else {
      CERR("[create::Create] ", "Front right cliff sensors not supported!");
      return false;
    }
  }

  bool Create::isVirtualWall() const {
    if (data->isValidPacketID(ID_VIRTUAL_WALL)) {
      return GET_DATA(ID_VIRTUAL_WALL);
    }
    else {
      CERR("[create::Create] ", "Virtual Wall sensor not supported!");
      return false;
    }
  }

  uint8_t Create::getDirtDetect() const {
    if (data->isValidPacketID(ID_DIRT_DETECT_LEFT)) {
      return GET_DATA(ID_DIRT_DETECT_LEFT);
    }
    else {
      CERR("[create::Create] ", "Dirt detector not supported!");
      return -1;
    }
  }

  uint8_t Create::getIROmni() const {
    if (data->isValidPacketID(ID_IR_OMNI)) {
      return GET_DATA(ID_IR_OMNI);
    }
    else {
      CERR("[create::Create] ", "Omni IR sensor not supported!");
      return -1;
    }
  }

  uint8_t Create::getIRLeft() const {
    if (data->isValidPacketID(ID_IR_LEFT)) {
      return GET_DATA(ID_IR_LEFT);
    }
    else {
      CERR("[create::Create] ", "Left IR sensor not supported!");
      return -1;
    }
  }

  uint8_t Create::getIRRight() const {
    if (data->isValidPacketID(ID_IR_RIGHT)) {
      return GET_DATA(ID_IR_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Right IR sensor not supported!");
      return -1;
    }
  }

  ChargingState Create::getChargingState() const {
    if (data->isValidPacketID(ID_CHARGE_STATE)) {
      uint8_t chargeState = GET_DATA(ID_CHARGE_STATE);
      assert(chargeState <= 5);
      return (ChargingState) chargeState;
    }
    else {
      CERR("[create::Create] ", "Charging state not supported!");
      return CHARGE_FAULT;
    }
  }

  bool Create::isCleanButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  // Not supported by any 600 series firmware
  bool Create::isClockButtonPressed() const {
    CERR("[create::Create] ", "Clock button is not supported!");
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x80) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  // Not supported by any 600 series firmware
  bool Create::isScheduleButtonPressed() const {
    CERR("[create::Create] ", "Schedule button is not supported!");
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x40) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isDayButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x20) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isHourButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x10) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isMinButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x08) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isDockButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  bool Create::isSpotButtonPressed() const {
    if (data->isValidPacketID(ID_BUTTONS)) {
      return (GET_DATA(ID_BUTTONS) & 0x02) != 0;
    }
    else {
      CERR("[create::Create] ", "Buttons not supported!");
      return false;
    }
  }

  float Create::getVoltage() const {
    if (data->isValidPacketID(ID_VOLTAGE)) {
      return (GET_DATA(ID_VOLTAGE) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Voltage sensor not supported!");
      return 0;
    }
  }

  float Create::getCurrent() const {
    if (data->isValidPacketID(ID_VOLTAGE)) {
      return (((int16_t)GET_DATA(ID_CURRENT)) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Current sensor not supported!");
      return 0;
    }
  }

  int8_t Create::getTemperature() const {
    if (data->isValidPacketID(ID_TEMP)) {
      return (int8_t) GET_DATA(ID_TEMP);
    }
    else {
      CERR("[create::Create] ", "Temperature sensor not supported!");
      return 0;
    }
  }

  float Create::getBatteryCharge() const {
    if (data->isValidPacketID(ID_CHARGE)) {
      return (GET_DATA(ID_CHARGE) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Battery charge not supported!");
      return 0;
    }
  }

  float Create::getBatteryCapacity() const {
    if (data->isValidPacketID(ID_CAPACITY)) {
      return (GET_DATA(ID_CAPACITY) / 1000.0);
    }
    else {
      CERR("[create::Create] ", "Battery capacity not supported!");
      return 0;
    }
  }

  bool Create::isLightBumperLeft() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperFrontLeft() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x02) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperCenterLeft() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperCenterRight() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x08) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperFrontRight() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x10) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  bool Create::isLightBumperRight() const {
    if (data->isValidPacketID(ID_LIGHT)) {
      return (GET_DATA(ID_LIGHT) & 0x20) != 0;
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return false;
    }
  }

  uint16_t Create::getLightSignalLeft() const {
    if (data->isValidPacketID(ID_LIGHT_LEFT)) {
      return GET_DATA(ID_LIGHT_LEFT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalFrontLeft() const {
    if (data->isValidPacketID(ID_LIGHT_FRONT_LEFT)) {
      return GET_DATA(ID_LIGHT_FRONT_LEFT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalCenterLeft() const {
    if (data->isValidPacketID(ID_LIGHT_CENTER_LEFT)) {
      return GET_DATA(ID_LIGHT_CENTER_LEFT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalRight() const {
    if (data->isValidPacketID(ID_LIGHT_RIGHT)) {
      return GET_DATA(ID_LIGHT_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalFrontRight() const {
    if (data->isValidPacketID(ID_LIGHT_FRONT_RIGHT)) {
      return GET_DATA(ID_LIGHT_FRONT_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  uint16_t Create::getLightSignalCenterRight() const {
    if (data->isValidPacketID(ID_LIGHT_CENTER_RIGHT)) {
      return GET_DATA(ID_LIGHT_CENTER_RIGHT);
    }
    else {
      CERR("[create::Create] ", "Light sensors not supported!");
      return 0;
    }
  }

  bool Create::isMovingForward() const {
    if (data->isValidPacketID(ID_STASIS)) {
      return GET_DATA(ID_STASIS) == 1;
    }
    else {
      CERR("[create::Create] ", "Stasis sensor not supported!");
      return false;
    }
  }

  bool Create::isSideBrushOvercurrent() const {
    if (data->isValidPacketID(ID_OVERCURRENTS)) {
      return (GET_DATA(ID_OVERCURRENTS) & 0x01) != 0;
    }
    else {
      CERR("[create::Create] ", "Overcurrent sensor not supported!");
      return false;
    }
  }

  bool Create::isMainBrushOvercurrent() const {
    if (data->isValidPacketID(ID_OVERCURRENTS)) {
      return (GET_DATA(ID_OVERCURRENTS) & 0x04) != 0;
    }
    else {
      CERR("[create::Create] ", "Overcurrent sensor not supported!");
      return false;
    }
  }

  bool Create::isWheelOvercurrent() const {
    if (data->isValidPacketID(ID_OVERCURRENTS)) {
      return (GET_DATA(ID_OVERCURRENTS) & 0x18) != 0;
    }
    else {
      CERR("[create::Create] ", "Overcurrent sensor not supported!");
      return false;
    }
  }

  float Create::getLeftWheelDistance() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return totalLeftDist;
  }

  float Create::getRightWheelDistance() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return totalRightDist;
  }

  float Create::getMeasuredLeftWheelVel() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return measuredLeftVel;
  }

  float Create::getMeasuredRightWheelVel() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return measuredRightVel;
  }

  float Create::getRequestedLeftWheelVel() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return requestedLeftVel;
  }

  float Create::getRequestedRightWheelVel() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return requestedRightVel;
  }

  void Create::setModeReportWorkaround(const bool& enable) {
    modeReportWorkaround = enable;
  }

  bool Create::getModeReportWorkaround() const {
    return modeReportWorkaround;
  }

  create::CreateMode Create::getMode() {
    if (data->isValidPacketID(ID_OI_MODE)) {
      if (modeReportWorkaround) {
        mode = (create::CreateMode) (GET_DATA(ID_OI_MODE) - 1);
      } else {
        mode = (create::CreateMode) GET_DATA(ID_OI_MODE);
      }
    }

    return mode;
  }

  Pose Create::getPose() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return pose;
  }

  Vel Create::getVel() const {
    std::lock_guard<std::mutex> lock(odometryMutex);
    return vel;
  }

  uint64_t Create::getNumCorruptPackets() const {
    return serial->getNumCorruptPackets();
  }

  uint64_t Create::getTotalPackets() const {
    return serial->getTotalPackets();
  }

} // end namespace
