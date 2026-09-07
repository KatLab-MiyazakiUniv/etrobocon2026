/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  正方形を利用してロボットの向きを補正するクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "ClockUtil.h"
#include "Logger.h"
#include "SocketClient.h"

namespace {

  /**
   * @brief 円周率
   */
  constexpr double PI = 3.14159265358979323846;

  /**
   * @brief 最大旋回Power
   */
  constexpr double MAX_TURNING_POWER = 30.0;

  /**
   * @brief 最低旋回Power
   *
   * 出力が小さすぎて車体が回らないことを防ぐ。
   */
  constexpr double MIN_TURNING_POWER = 8.0;

  /**
   * @brief 最大補正回数
   *
   * 異常時の無限ループを防止する。
   */
  constexpr int MAX_ADJUSTMENT_COUNT = 300;

  /**
   * @brief 補正ループ待機時間[ms]
   */
  constexpr int LOOP_SLEEP_TIME = 1;

}  // namespace

SquareAngleAdjustment::SquareAngleAdjustment(
    Robot& _robot, const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
    const Pid::PidGain& _pidGain, double _angleTolerance)
  : robot(_robot),
    squareDetectionRequest(_squareDetectionRequest),
    anglePid(_pidGain.kp, _pidGain.ki, _pidGain.kd, 0.0),
    angleTolerance(_angleTolerance)
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
                    "SquareAngleAdjustment: "
                    "start tolerance=%.2f",
                    angleTolerance);

  /*
   * PID目標値は0度。
   */
  anglePid.prepare();

  SocketClient& client = robot.getCameraSocketClientInstance();

  for(int count = 0; count < MAX_ADJUSTMENT_COUNT; ++count) {
    // =====================================================
    // 正方形検出
    // =====================================================

    CameraServer::SquareDetectorResponse response{};

    const bool success = client.executeSquareDetection(squareDetectionRequest, response);

    // =====================================================
    // 通信失敗
    // =====================================================

    if(!success) {
      stop();

      Logger::warning("SquareAngleAdjustment: "
                      "detection communication failed "
                      "-> skip");

      return false;
    }

    // =====================================================
    // 正方形未検出
    //
    // 探し回ったりせず、その場で補正を中止する。
    // =====================================================

    if(!response.wasDetected) {
      stop();

      Logger::warning("SquareAngleAdjustment: "
                      "square not detected "
                      "-> skip");

      return false;
    }

    // =====================================================
    // 正方形角度計算
    // =====================================================

    const double currentAngle = calculateSquareAngle(response);

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "angle=%.2f deg",
                      currentAngle);

    // =====================================================
    // 補正完了
    // =====================================================

    if(std::abs(currentAngle) <= angleTolerance) {
      stop();

      Logger::printfLog(Logger::INFO,
                        "SquareAngleAdjustment: "
                        "completed angle=%.2f deg",
                        currentAngle);

      return true;
    }

    // =====================================================
    // PID
    // =====================================================

    double turningPower = anglePid.calculatePid(currentAngle) * -1.0;

    /*
     * 最大Power制限
     */
    turningPower = std::max(-MAX_TURNING_POWER, std::min(turningPower, MAX_TURNING_POWER));

    /*
     * 最低Power保証
     */
    if(std::abs(turningPower) < MIN_TURNING_POWER) {
      if(turningPower >= 0.0) {
        turningPower = MIN_TURNING_POWER;

      } else {
        turningPower = -MIN_TURNING_POWER;
      }
    }

    // =====================================================
    // その場回転
    // =====================================================

    const double rightPower = -turningPower;

    const double leftPower = turningPower;

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "angle=%.2f "
                      "turn=%.2f "
                      "right=%.2f "
                      "left=%.2f",
                      currentAngle, turningPower, rightPower, leftPower);

    robot.getWheelMotorControllerInstance().setRightPower(rightPower);

    robot.getWheelMotorControllerInstance().setLeftPower(leftPower);

    ClockUtil::sleep(LOOP_SLEEP_TIME);
  }

  // =====================================================
  // 最大回数到達
  // =====================================================

  stop();

  Logger::warning("SquareAngleAdjustment: "
                  "adjustment limit reached");

  return false;
}

double SquareAngleAdjustment::calculateSquareAngle(
    const CameraServer::SquareDetectorResponse& response) const
{
  /*
   * cornersの順番に依存しないようにする。
   *
   * Y座標が小さい2点を
   * 正方形の上側2点として扱う。
   */

  std::array<int, 4> indices = { 0, 1, 2, 3 };

  std::sort(indices.begin(), indices.end(),
            [&response](int a, int b) { return response.corners[a].y < response.corners[b].y; });

  const auto& point1 = response.corners[indices[0]];

  const auto& point2 = response.corners[indices[1]];

  /*
   * Xが小さい方を左上、
   * Xが大きい方を右上とする。
   */
  const auto& topLeft = point1.x < point2.x ? point1 : point2;

  const auto& topRight = point1.x < point2.x ? point2 : point1;

  const double deltaX = static_cast<double>(topRight.x - topLeft.x);

  const double deltaY = static_cast<double>(topRight.y - topLeft.y);

  const double angleRad = std::atan2(deltaY, deltaX);

  return angleRad * 180.0 / PI;
}

void SquareAngleAdjustment::stop()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}