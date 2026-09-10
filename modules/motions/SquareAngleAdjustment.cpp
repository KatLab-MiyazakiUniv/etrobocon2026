/**
 * @file   SquareAngleAdjustment.cpp
 * @brief  正方形の中心位置から角度を計算し、RelativeRotationで向きを補正するクラス
 * @author yutaro-1214
 */

#include "SquareAngleAdjustment.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "ClockUtil.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "SocketClient.h"
#include "SystemInfo.h"

namespace {


  /**
   * @brief 正方形検出を再試行する最大回数
   *
   * カメラ画像によって一時的に検出できない場合があるため、
   * 1回失敗しただけでは終了しない。
   */
  constexpr int MAX_DETECTION_RETRY_COUNT = 3;

  /**
   * @brief 正方形検出再試行までの待機時間[ms]
   */
  constexpr int DETECTION_RETRY_WAIT_TIME = 50;

  /**
   * @brief RelativeRotation後の停止待機時間[ms]
   */
  constexpr int AFTER_ROTATION_WAIT_TIME = 100;

  /**
   * @brief 1回のSquare補正で許可する最大角度[deg]
   *
   * 誤検出によって大きく回頭してしまうことを防ぐ。
   */
  constexpr double MAX_CORRECTION_ANGLE = 25.0;

  /**
   * @brief 小さすぎる回頭を行わないための最低角度[deg]
   */
  constexpr double MIN_CORRECTION_ANGLE = 0.5;

}  // namespace

SquareAngleAdjustment::SquareAngleAdjustment(
    Robot& _robot,
    const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
    const Pid::PidGain& _rotationPid,
    double _centerTolerance,
    double _horizontalFovDeg,
    double _rotationTolerance)
  : robot(_robot),
    squareDetectionRequest(_squareDetectionRequest),
    rotationPid(_rotationPid),
    centerTolerance(_centerTolerance),
    horizontalFovDeg(_horizontalFovDeg),
    rotationTolerance(_rotationTolerance)
{
  LOG_CREATE("SquareAngleAdjustment");

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "rotationPid=(%.6f, %.6f, %.6f) "
      "centerTolerance=%.2f "
      "horizontalFov=%.2f "
      "rotationTolerance=%.2f",
      rotationPid.kp,
      rotationPid.ki,
      rotationPid.kd,
      centerTolerance,
      horizontalFovDeg,
      rotationTolerance);
}

SquareAngleAdjustment::~SquareAngleAdjustment()
{
  LOG_DESTROY("SquareAngleAdjustment");
}

bool SquareAngleAdjustment::run()
{
  Logger::info(
      "SquareAngleAdjustment: ========== START ==========");

  stop();

  SocketClient& client =
      robot.getCameraSocketClientInstance();

  CameraServer::SquareDetectorResponse response {};

  bool detected = false;

  // =====================================================
  // 正方形検出
  // =====================================================

  for(int attempt = 0;
      attempt < MAX_DETECTION_RETRY_COUNT;
      attempt++) {

    CameraServer::SquareDetectorRequest request =
        squareDetectionRequest;

    /*
     * Straight後にresetTracking=trueになっている場合でも、
     * リセットするのは最初の検出だけ。
     *
     * 2回目以降の検出では追跡状態を維持する。
     */
    request.resetTracking =
        squareDetectionRequest.resetTracking
        && attempt == 0;

    Logger::printfLog(
        Logger::INFO,
        "SquareAngleAdjustment: "
        "detection attempt=%d resetTracking=%s",
        attempt + 1,
        request.resetTracking
            ? "true"
            : "false");

    response = {};

    const bool communicationSuccess =
        client.executeSquareDetection(
            request,
            response);

    if(!communicationSuccess) {

      Logger::warning(
          "SquareAngleAdjustment: "
          "Square detection communication failed");

      if(attempt
         < MAX_DETECTION_RETRY_COUNT - 1) {

        ClockUtil::sleep(
            DETECTION_RETRY_WAIT_TIME);

        continue;
      }

      stop();

      Logger::warning(
          "SquareAngleAdjustment: "
          "detection retry limit reached");

      return false;
    }

    if(!response.wasDetected) {

      Logger::warning(
          "SquareAngleAdjustment: "
          "Square not detected");

      if(attempt
         < MAX_DETECTION_RETRY_COUNT - 1) {

        ClockUtil::sleep(
            DETECTION_RETRY_WAIT_TIME);

        continue;
      }

      stop();

      Logger::warning(
          "SquareAngleAdjustment: "
          "Square was not detected");

      return false;
    }

    detected = true;

    break;
  }

  if(!detected) {

    stop();

    return false;
  }

  // =====================================================
  // 正方形中心を計算
  // =====================================================

  const double centerX =
      calculateCenterX(
          response);

  const double imageCenterX =
      CAM_MAX_WIDTH / 2.0;

  const double pixelError =
      calculatePixelError(
          response);

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "squareCenterX=%.2f "
      "imageCenterX=%.2f "
      "pixelError=%.2f",
      centerX,
      imageCenterX,
      pixelError);

  // =====================================================
  // 既に中央の場合
  // =====================================================

  if(std::abs(pixelError)
     <= centerTolerance) {

    stop();

    Logger::printfLog(
        Logger::INFO,
        "SquareAngleAdjustment: "
        "already centered "
        "error=%.2f tolerance=%.2f",
        pixelError,
        centerTolerance);

    Logger::info(
        "SquareAngleAdjustment: ========== SUCCESS ==========");

    return true;
  }

  // =====================================================
  // pixel → 角度
  // =====================================================

  double rotationAngle =
      pixelErrorToRotationAngle(
          pixelError);

  // =====================================================
  // 最大補正角度制限
  // =====================================================

  rotationAngle =
      std::max(
          -MAX_CORRECTION_ANGLE,
          std::min(
              rotationAngle,
              MAX_CORRECTION_ANGLE));

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "pixelError=%.2f -> rotationAngle=%.2f deg",
      pixelError,
      rotationAngle);

  // =====================================================
  // 小さすぎる角度なら終了
  // =====================================================

  if(std::abs(rotationAngle)
     < MIN_CORRECTION_ANGLE) {

    stop();

    Logger::printfLog(
        Logger::INFO,
        "SquareAngleAdjustment: "
        "rotation skipped because angle is too small "
        "%.2f deg",
        rotationAngle);

    return true;
  }

  // =====================================================
  // RelativeRotationによる角度補正
  // =====================================================

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "RelativeRotation START %.2f deg",
      rotationAngle);

  rotate(
      rotationAngle);

  stop();

  ClockUtil::sleep(
      AFTER_ROTATION_WAIT_TIME);

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "RelativeRotation FINISHED %.2f deg",
      rotationAngle);

  Logger::info(
      "SquareAngleAdjustment: ========== SUCCESS ==========");

  return true;
}

double SquareAngleAdjustment::calculateCenterX(
    const CameraServer::SquareDetectorResponse& response) const
{
  double centerX = 0.0;

  for(uint32_t i = 0;
      i < CameraServer::SQUARE_CORNER_COUNT;
      i++) {

    centerX +=
        static_cast<double>(
            response.corners[i].x);
  }

  centerX /=
      static_cast<double>(
          CameraServer::SQUARE_CORNER_COUNT);

  return centerX;
}

double SquareAngleAdjustment::calculatePixelError(
    const CameraServer::SquareDetectorResponse& response) const
{
  const double centerX =
      calculateCenterX(
          response);

  const double imageCenterX =
      CAM_MAX_WIDTH / 2.0;

  return centerX
         - imageCenterX;
}

double SquareAngleAdjustment::pixelErrorToRotationAngle(
    double pixelError) const
{
  // =====================================================
  // 水平画角をradへ変換
  // =====================================================

  const double horizontalFovRad =
      horizontalFovDeg
      * PI
      / 180.0;

  // =====================================================
  // カメラの焦点距離[pixel]を計算
  //
  //           imageWidth / 2
  // fx = -------------------------
  //       tan(horizontalFov / 2)
  // =====================================================

  const double halfImageWidth =
      CAM_MAX_WIDTH / 2.0;

  const double focalLengthPixel =
      halfImageWidth
      / std::tan(
          horizontalFovRad / 2.0);

  // =====================================================
  // pixel差からカメラに対する角度を求める
  //
  // angle = atan(pixelError / fx)
  // =====================================================

  const double cameraAngleRad =
      std::atan(
          pixelError
          / focalLengthPixel);

  const double cameraAngleDeg =
      cameraAngleRad
      * 180.0
      / PI;

  /*
   * 画像上で
   *
   * pixelError > 0
   *
   * の場合、正方形はロボットから見て右側にある。
   *
   * 現在のRouteFollower / RelativeRotationの符号では、
   * 右回頭を負の角度として扱うため反転する。
   */
  const double rotationAngle =
      -cameraAngleDeg;

  Logger::printfLog(
      Logger::INFO,
      "SquareAngleAdjustment: "
      "pixelToAngle "
      "pixel=%.2f "
      "focal=%.2f "
      "cameraAngle=%.2f "
      "rotationAngle=%.2f",
      pixelError,
      focalLengthPixel,
      cameraAngleDeg,
      rotationAngle);

  return rotationAngle;
}

void SquareAngleAdjustment::rotate(
    double angle)
{
  auto condition =
      std::make_unique<RelativeAngleCondition>(
          robot,
          angle,
          rotationTolerance);

  RelativeRotation rotation(
      robot,
      std::move(condition),
      rotationPid,
      angle);

  rotation.run();
}

void SquareAngleAdjustment::stop()
{
  robot
      .getWheelMotorControllerInstance()
      .stopBoth();
}