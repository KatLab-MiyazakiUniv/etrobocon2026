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
#include "SystemInfo.h"

namespace {

  /**
   * @brief 最大旋回Power
   */
  constexpr double MAX_TURNING_POWER = 40.0;

  /**
   * @brief 最低旋回Power
   *
   * 出力が小さすぎて車体が回らないことを防ぐ。
   */
  constexpr double MIN_TURNING_POWER = 12.0;

  /**
   * @brief 最大補正回数
   *
   * 異常時の無限ループを防止する。
   */
  constexpr int MAX_ADJUSTMENT_COUNT = 300;

  /**
   * @brief 1回の補正でモータを回す時間[ms]
   */
  constexpr int ROTATION_PULSE_TIME = 80;

  /**
   * @brief 回転停止後の待機時間[ms]
   *
   * 車体とカメラ画像が安定してから
   * 次の正方形検出を行う。
   */
  constexpr int DETECTION_SETTLING_TIME = 200;

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
                    "START tolerance=%.2f",
                    angleTolerance);

  // =====================================================
  // PID初期化
  // =====================================================

  anglePid.prepare();

  SocketClient& client = robot.getCameraSocketClientInstance();

  // =====================================================
  // 補正ループ
  // =====================================================

  for(int count = 0; count < MAX_ADJUSTMENT_COUNT; ++count) {
    // ===================================================
    // 検出前に必ず停止
    // ===================================================

    stop();

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "detection count=%d",
                      count);

    // ===================================================
    // 正方形検出
    // ===================================================

    CameraServer::SquareDetectorResponse response{};

    const bool success = client.executeSquareDetection(squareDetectionRequest, response);

    // ===================================================
    // 通信失敗
    // ===================================================

    if(!success) {
      stop();

      Logger::warning("SquareAngleAdjustment: "
                      "detection communication failed "
                      "-> SKIP");

      return false;
    }

    // ===================================================
    // 正方形未検出
    //
    // この回の角度補正のみ飛ばす。
    // ===================================================

    if(!response.wasDetected) {
      stop();

      Logger::warning("SquareAngleAdjustment: "
                      "square NOT DETECTED "
                      "-> SKIP THIS CORRECTION");

      return false;
    }

    // ===================================================
    // corners確認
    // ===================================================

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "corners "
                      "0=(%d,%d) "
                      "1=(%d,%d) "
                      "2=(%d,%d) "
                      "3=(%d,%d)",
                      response.corners[0].x, response.corners[0].y, response.corners[1].x,
                      response.corners[1].y, response.corners[2].x, response.corners[2].y,
                      response.corners[3].x, response.corners[3].y);

    // ===================================================
    // 正方形角度計算
    // ===================================================

    const double currentAngle = calculateSquareAngle(response);

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "DETECTED "
                      "angle=%.3f deg "
                      "tolerance=%.3f deg",
                      currentAngle, angleTolerance);

    // ===================================================
    // 補正終了判定
    // ===================================================

    if(std::abs(currentAngle) <= angleTolerance) {
      Logger::printfLog(Logger::INFO,
                        "SquareAngleAdjustment: "
                        "SKIP MOTOR "
                        "|angle|=%.3f <= tolerance=%.3f",
                        std::abs(currentAngle), angleTolerance);

      stop();

      return true;
    }

    // ===================================================
    // ここまで来たら実際に補正する
    // ===================================================

    Logger::info("SquareAngleAdjustment: "
                 "START MOTOR CORRECTION");

    // ===================================================
    // PID計算
    // ===================================================

    double turningPower = anglePid.calculatePid(currentAngle) * -1.0;

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "PID BEFORE LIMIT "
                      "angle=%.3f "
                      "power=%.3f",
                      currentAngle, turningPower);

    // ===================================================
    // 最大Power制限
    // ===================================================

    turningPower = std::max(-MAX_TURNING_POWER, std::min(turningPower, MAX_TURNING_POWER));

    // ===================================================
    // 最低Power保証
    // ===================================================

    if(std::abs(turningPower) < MIN_TURNING_POWER) {
      if(turningPower >= 0.0) {
        turningPower = MIN_TURNING_POWER;

      } else {
        turningPower = -MIN_TURNING_POWER;
      }
    }

    // ===================================================
    // モータ出力
    // ===================================================

    const double rightPower = -turningPower;

    const double leftPower = turningPower;

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "MOTOR COMMAND "
                      "angle=%.3f "
                      "turn=%.3f "
                      "right=%.3f "
                      "left=%.3f",
                      currentAngle, turningPower, rightPower, leftPower);

    robot.getWheelMotorControllerInstance().setRightPower(rightPower);

    robot.getWheelMotorControllerInstance().setLeftPower(leftPower);

    // ===================================================
    // 短時間だけ回転
    // ===================================================

    ClockUtil::sleep(ROTATION_PULSE_TIME);

    // ===================================================
    // 回転停止
    // ===================================================

    stop();

    Logger::info("SquareAngleAdjustment: "
                 "rotation pulse finished");

    // ===================================================
    // 次の検出まで待機
    // ===================================================

    ClockUtil::sleep(DETECTION_SETTLING_TIME);
  }

  // =====================================================
  // 最大補正回数到達
  // =====================================================

  stop();

  Logger::warning("SquareAngleAdjustment: "
                  "adjustment count limit reached");

  return false;
}

double SquareAngleAdjustment::calculateSquareAngle(
    const CameraServer::SquareDetectorResponse& response) const
{
  // =====================================================
  // cornersの並び順を整理する
  // =====================================================

  std::array<int, 4> indices = { 0, 1, 2, 3 };

  /*
   * Y座標が小さい順に並べる。
   */
  std::sort(indices.begin(), indices.end(),
            [&response](int a, int b) { return response.corners[a].y < response.corners[b].y; });

  /*
   * 上側にある2点を取得する。
   */
  const auto& point1 = response.corners[indices[0]];

  const auto& point2 = response.corners[indices[1]];

  // =====================================================
  // 左上・右上を決定
  // =====================================================

  const auto& topLeft = point1.x < point2.x ? point1 : point2;

  const auto& topRight = point1.x < point2.x ? point2 : point1;

  // =====================================================
  // ログ
  // =====================================================

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "topLeft=(%d,%d) "
                    "topRight=(%d,%d)",
                    topLeft.x, topLeft.y, topRight.x, topRight.y);

  // =====================================================
  // 上辺の傾きを計算
  // =====================================================

  const double deltaX = static_cast<double>(topRight.x - topLeft.x);

  const double deltaY = static_cast<double>(topRight.y - topLeft.y);

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "deltaX=%.2f "
                    "deltaY=%.2f",
                    deltaX, deltaY);

  const double angleRad = std::atan2(deltaY, deltaX);

  const double angleDeg = angleRad * 180.0 / PI;

  Logger::printfLog(Logger::INFO,
                    "SquareAngleAdjustment: "
                    "calculated angle=%.3f deg",
                    angleDeg);

  return angleDeg;
}

void SquareAngleAdjustment::stop()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}