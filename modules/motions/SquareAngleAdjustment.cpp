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
   *
   * 連続回転させず、
   * 短時間だけ回転させる。
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
                    "start tolerance=%.2f",
                    angleTolerance);

  // =====================================================
  // PID初期化
  // =====================================================

  /*
   * PIDの目標角度は0度。
   */
  anglePid.prepare();

  SocketClient& client = robot.getCameraSocketClientInstance();

  // =====================================================
  // 角度補正ループ
  // =====================================================

  for(int count = 0; count < MAX_ADJUSTMENT_COUNT; ++count) {
    // ===================================================
    // 検出前に必ず停止
    // ===================================================

    /*
     * 回転中の画像では正方形検出が
     * 不安定になる可能性があるため、
     * 正方形を検出するときは必ず停止状態にする。
     */
    stop();

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
                      "-> skip");

      return false;
    }

    // ===================================================
    // 正方形未検出
    //
    // この回の補正だけスキップする。
    // ===================================================

    if(!response.wasDetected) {
      stop();

      Logger::warning("SquareAngleAdjustment: "
                      "square not detected "
                      "-> skip this correction");

      return false;
    }

    // ===================================================
    // 正方形角度計算
    // ===================================================

    const double currentAngle = calculateSquareAngle(response);

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "detected=true "
                      "angle=%.2f deg "
                      "tolerance=%.2f deg",
                      currentAngle, angleTolerance);

    // ===================================================
    // 補正終了判定
    // ===================================================

    if(std::abs(currentAngle) <= angleTolerance) {
      stop();

      Logger::printfLog(Logger::INFO,
                        "SquareAngleAdjustment: "
                        "completed "
                        "angle=%.2f deg",
                        currentAngle);

      return true;
    }

    // ===================================================
    // PID計算
    // ===================================================

    double turningPower = anglePid.calculatePid(currentAngle) * -1.0;

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "PID raw=%.2f",
                      turningPower);

    // ===================================================
    // 最大Power制限
    // ===================================================

    turningPower = std::max(-MAX_TURNING_POWER, std::min(turningPower, MAX_TURNING_POWER));

    // ===================================================
    // 最低Power保証
    // ===================================================

    /*
     * PID出力が小さすぎると、
     * 静止摩擦によって車体が回らない可能性がある。
     */
    if(std::abs(turningPower) < MIN_TURNING_POWER) {
      if(turningPower >= 0.0) {
        turningPower = MIN_TURNING_POWER;

      } else {
        turningPower = -MIN_TURNING_POWER;
      }
    }

    // ===================================================
    // その場回転用Power
    // ===================================================

    const double rightPower = -turningPower;

    const double leftPower = turningPower;

    Logger::printfLog(Logger::INFO,
                      "SquareAngleAdjustment: "
                      "angle=%.2f "
                      "turn=%.2f "
                      "right=%.2f "
                      "left=%.2f",
                      currentAngle, turningPower, rightPower, leftPower);

    // ===================================================
    // 短時間だけその場回転
    // ===================================================

    robot.getWheelMotorControllerInstance().setRightPower(rightPower);

    robot.getWheelMotorControllerInstance().setLeftPower(leftPower);

    /*
     * 連続回転させず、
     * ROTATION_PULSE_TIMEだけ回転する。
     */
    ClockUtil::sleep(ROTATION_PULSE_TIME);

    // ===================================================
    // 回転終了
    // ===================================================

    stop();

    Logger::info("SquareAngleAdjustment: "
                 "rotation pulse finished");

    // ===================================================
    // 車体・画像安定待ち
    // ===================================================

    /*
     * モータ停止直後は車体の揺れや
     * モーションブラーが残る可能性がある。
     *
     * 次の正方形検出は、
     * DETECTION_SETTLING_TIME待ってから行う。
     */
    ClockUtil::sleep(DETECTION_SETTLING_TIME);

    /*
     * 次のforループへ進み、
     * 停止した状態で再び正方形を検出する。
     */
  }

  // =====================================================
  // 最大補正回数到達
  // =====================================================

  stop();

  Logger::warning("SquareAngleAdjustment: "
                  "adjustment count limit reached "
                  "-> skip");

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

  /*
   * Y座標が小さい2点を取得する。
   */
  const auto& point1 = response.corners[indices[0]];

  const auto& point2 = response.corners[indices[1]];

  /*
   * X座標が小さい方を左上、
   * X座標が大きい方を右上とする。
   */
  const auto& topLeft = point1.x < point2.x ? point1 : point2;

  const auto& topRight = point1.x < point2.x ? point2 : point1;

  // =====================================================
  // 正方形上辺の傾きを計算
  // =====================================================

  const double deltaX = static_cast<double>(topRight.x - topLeft.x);

  const double deltaY = static_cast<double>(topRight.y - topLeft.y);

  const double angleRad = std::atan2(deltaY, deltaX);

  /*
   * rad → deg
   *
   * PIはSystemInfo.hの定義を使用する。
   */
  return angleRad * 180.0 / PI;
}

void SquareAngleAdjustment::stop()
{
  robot.getWheelMotorControllerInstance().stopBoth();
}