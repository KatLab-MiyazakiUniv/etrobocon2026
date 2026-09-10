/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  正方形の中心を利用してロボットの向きを補正するクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "ClockUtil.h"
#include "Logger.h"
#include "SocketClient.h"
#include "SystemInfo.h"

namespace {

/**
 * @brief 最大旋回Power
 */
constexpr double MAX_TURNING_POWER = 40.0;

/**
 * @brief 最低旋回Power
 */
constexpr double MIN_TURNING_POWER = 12.0;

/**
 * @brief 最大補正回数
 */
constexpr int MAX_ADJUSTMENT_COUNT = 300;

/**
 * @brief モータ回転時間[ms]
 */
constexpr int ROTATION_PULSE_TIME = 80;

/**
 * @brief 停止後待機時間[ms]
 */
constexpr int DETECTION_SETTLING_TIME = 200;

}  // namespace

SquareAngleAdjustment::SquareAngleAdjustment(
    Robot& _robot,
    const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
    const Pid::PidGain& _pidGain,
    double _centerTolerance)
  : robot(_robot),
    squareDetectionRequest(_squareDetectionRequest),
    centerPid(_pidGain.kp, _pidGain.ki, _pidGain.kd, 0.0),
    centerTolerance(_centerTolerance)
{
  LOG_CREATE("SquareAngleAdjustment");
}

SquareAngleAdjustment::~SquareAngleAdjustment()
{
  LOG_DESTROY("SquareAngleAdjustment");
}

bool SquareAngleAdjustment::run()
{
  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment START tolerance=%.2f pixel",
                    centerTolerance);

  centerPid.prepare();

  SocketClient& client = robot.getCameraSocketClientInstance();

  for(int count = 0; count < MAX_ADJUSTMENT_COUNT; count++) {

    stop();

    CameraServer::SquareDetectorResponse response{};

    const bool success =
        client.executeSquareDetection(squareDetectionRequest, response);

    if(!success) {
      stop();
      Logger::warning("Square detection communication failed");
      return false;
    }

    if(!response.wasDetected) {
      stop();
      Logger::warning("Square not detected");
      return false;
    }

    const double currentError = calculateCenterError(response);

    Logger::printfLog(Logger::INFO,
                      "Center error = %.2f pixel",
                      currentError);

    if(std::abs(currentError) <= centerTolerance) {
      stop();
      Logger::info("Center aligned");
      return true;
    }

    double turningPower = centerPid.calculatePid(currentError) * -1.0;

    turningPower =
        std::max(-MAX_TURNING_POWER,
                 std::min(turningPower, MAX_TURNING_POWER));

    if(std::abs(turningPower) < MIN_TURNING_POWER) {
      turningPower =
          (turningPower >= 0.0) ? MIN_TURNING_POWER : -MIN_TURNING_POWER;
    }

    robot.getWheelMotorControllerInstance().setRightPower(-turningPower);
    robot.getWheelMotorControllerInstance().setLeftPower(turningPower);

    ClockUtil::sleep(ROTATION_PULSE_TIME);

    stop();

    ClockUtil::sleep(DETECTION_SETTLING_TIME);
  }

  stop();

  Logger::warning("Adjustment count limit reached");

  return false;
}

double SquareAngleAdjustment::calculateCenterError(
    const CameraServer::SquareDetectorResponse& response) const
{
  double centerX = 0.0;
  double centerY = 0.0;

  for(int i = 0; i < 4; i++) {
    centerX += response.corners[i].x;
    centerY += response.corners[i].y;
  }

  centerX /= 4.0;
  centerY /= 4.0;

  constexpr double IMAGE_CENTER_X = CAM_MAX_WIDTH / 2.0;

  const double error = centerX - IMAGE_CENTER_X;

  Logger::printfLog(
      Logger::INFO,
      "Square center=(%.1f, %.1f) imageCenterX=%.1f error=%.1f",
      centerX,
      centerY,
      IMAGE_CENTER_X,
      error);

  return error;
}

void SquareAngleAdjustment::stop()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}