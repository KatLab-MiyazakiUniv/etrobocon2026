/**
 * @file   CameraTracking.cpp
 * @brief  カメラを使ったPID走行クラス
 * @author sadomiya-sousi HaruArima08
 */

#include "CameraTracking.h"

#include <algorithm>
#include <utility>

// =====================================================
// 色領域検出用コンストラクタ
// =====================================================

CameraTracking::CameraTracking(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    double _targetSpeed,
    int _targetXCoordinate,
    const Pid::PidGain& _pidGain,
    const CameraServer::ColorRegionDetectorRequest& _colorDetectionRequest,
    bool _isStopMotorPower)
  : BaseMotion(
        _robot,
        std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(
        DetectionMode::COLOR_REGION),
    colorDetectionRequest(
        _colorDetectionRequest),
    qrDetectionRequest(),
    squareDetectionRequest(),
    isStopMotorPower(
        _isStopMotorPower),
    speedCalculator(
        _robot,
        _targetSpeed),
    cameraPid(
        _pidGain.kp,
        _pidGain.ki,
        _pidGain.kd,
        _targetXCoordinate)
{
  LOG_CREATE("CameraTracking");
}

// =====================================================
// QRコード検出用コンストラクタ
// =====================================================

CameraTracking::CameraTracking(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    double _targetSpeed,
    int _targetXCoordinate,
    const Pid::PidGain& _pidGain,
    const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
    bool _isStopMotorPower)
  : BaseMotion(
        _robot,
        std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(
        DetectionMode::QR_CODE),
    colorDetectionRequest(),
    qrDetectionRequest(
        _qrDetectionRequest),
    squareDetectionRequest(),
    isStopMotorPower(
        _isStopMotorPower),
    speedCalculator(
        _robot,
        _targetSpeed),
    cameraPid(
        _pidGain.kp,
        _pidGain.ki,
        _pidGain.kd,
        _targetXCoordinate)
{
  LOG_CREATE("CameraTracking");
}

// =====================================================
// 正方形検出用コンストラクタ
// =====================================================

CameraTracking::CameraTracking(
    Robot& _robot,
    std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    double _targetSpeed,
    int _targetXCoordinate,
    const Pid::PidGain& _pidGain,
    const CameraServer::SquareDetectorRequest& _squareDetectionRequest,
    bool _isStopMotorPower)
  : BaseMotion(
        _robot,
        std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(
        DetectionMode::SQUARE_DETECTION),
    colorDetectionRequest(),
    qrDetectionRequest(),
    squareDetectionRequest(
        _squareDetectionRequest),
    isStopMotorPower(
        _isStopMotorPower),
    speedCalculator(
        _robot,
        _targetSpeed),
    cameraPid(
        _pidGain.kp,
        _pidGain.ki,
        _pidGain.kd,
        _targetXCoordinate)
{
  LOG_CREATE("CameraTracking");
}

CameraTracking::~CameraTracking()
{
  LOG_DESTROY("CameraTracking");
}

bool CameraTracking::canStart()
{
  if(targetSpeed == 0.0) {

    Logger::error(
        "CameraTracking:目標スピードが0です");

    return false;
  }

  return true;
}

void CameraTracking::prepare()
{
  cameraPid.prepare();

  Logger::printfLog(
      Logger::INFO,
      "CameraTracking: start targetSpeed=%.2f targetX=%d mode=%d",
      targetSpeed,
      targetXCoordinate,
      static_cast<int>(detectionMode));
}

void CameraTracking::executeStep()
{
  // =====================================================
  // 基本モータPower
  // =====================================================

  const double baseRightPower =
      speedCalculator.calculateRightMotorPower();

  const double baseLeftPower =
      speedCalculator.calculateLeftMotorPower();

  SocketClient& client =
      robot.getCameraSocketClientInstance();

  bool success = false;
  bool wasDetected = false;

  double currentX = 0.0;

  // =====================================================
  // 色領域検出
  // =====================================================

  if(detectionMode
     == DetectionMode::COLOR_REGION) {

    CameraServer::ColorRegionDetectorResponse response {};

    success =
        client.executeColorRegionDetection(
            colorDetectionRequest,
            response);

    if(success) {

      wasDetected =
          response.result.wasDetected;

      if(wasDetected) {

        currentX =
            (response.result.topLeft.x
             + response.result.bottomRight.x)
            / 2.0;
      }
    }

  // =====================================================
  // QRコード検出
  // =====================================================

  } else if(detectionMode
            == DetectionMode::QR_CODE) {

    CameraServer::QrCodeDetectorResponse response {};

    success =
        client.executeQrCodeDetection(
            qrDetectionRequest,
            response);

    if(success) {

      wasDetected =
          response.wasDetected;

      if(wasDetected) {

        double sumX = 0.0;

        for(const auto& corner :
            response.corners) {

          sumX += corner.x;
        }

        currentX =
            sumX
            / CameraServer::QR_CODE_CORNER_COUNT;
      }
    }

  // =====================================================
  // 正方形検出
  // =====================================================

  } else if(detectionMode
            == DetectionMode::SQUARE_DETECTION) {

    CameraServer::SquareDetectorResponse response {};

    success =
        client.executeSquareDetection(
            squareDetectionRequest,
            response);

    if(success) {

      wasDetected =
          response.wasDetected;

      if(wasDetected) {

        double sumX = 0.0;
        double sumY = 0.0;

        for(const auto& corner :
            response.corners) {

          sumX += corner.x;
          sumY += corner.y;
        }

        currentX =
            sumX
            / CameraServer::SQUARE_CORNER_COUNT;

        const double currentY =
            sumY
            / CameraServer::SQUARE_CORNER_COUNT;

        const double errorX =
            currentX
            - static_cast<double>(
                targetXCoordinate);

        /*
         * 正方形を検出できているときのログ。
         */
        Logger::printfLog(
            Logger::INFO,
            "CameraTracking: SQUARE DETECTED "
            "center=(%.2f, %.2f) "
            "targetX=%d errorX=%.2f",
            currentX,
            currentY,
            targetXCoordinate,
            errorX);

      } else {

        /*
         * 通信には成功しているが、
         * 正方形が画像に存在しなかった場合。
         */
        Logger::warning(
            "CameraTracking: SQUARE NOT DETECTED");
      }
    }

  } else {

    Logger::error(
        "CameraTracking: invalid detection mode");

    return;
  }

  // =====================================================
  // 通信失敗
  // =====================================================

  if(!success) {

    Logger::warning(
        "CameraTracking: detection communication failed");

    return;
  }

  // =====================================================
  // 検出失敗
  // =====================================================

  if(!wasDetected) {

    /*
     * 正方形モードの場合は、
     * 一時的に見失っても通常直進する。
     *
     * そうしないと停止状態のまま
     * DistanceConditionが終了しない可能性がある。
     */
    if(detectionMode
       == DetectionMode::SQUARE_DETECTION) {

      robot
          .getWheelMotorControllerInstance()
          .setRightPower(
              baseRightPower);

      robot
          .getWheelMotorControllerInstance()
          .setLeftPower(
              baseLeftPower);

      Logger::printfLog(
          Logger::WARNING,
          "CameraTracking: square lost "
          "-> straight right=%.2f left=%.2f",
          baseRightPower,
          baseLeftPower);

      return;
    }

    Logger::warning(
        "CameraTracking:検出対象が検出できませんでした");

    return;
  }

  // =====================================================
  // PID角度補正
  // =====================================================

  const double turningPower =
      cameraPid.calculatePid(
          currentX)
      * -1.0;

  const double rightPower =
      baseRightPower > 0.0
          ? std::max(
                baseRightPower
                    - turningPower,
                0.0)
          : std::min(
                baseRightPower
                    + turningPower,
                0.0);

  const double leftPower =
      baseLeftPower > 0.0
          ? std::max(
                baseLeftPower
                    + turningPower,
                0.0)
          : std::min(
                baseLeftPower
                    - turningPower,
                0.0);

  // =====================================================
  // 正方形モードのPIDログ
  // =====================================================

  if(detectionMode
     == DetectionMode::SQUARE_DETECTION) {

    Logger::printfLog(
        Logger::INFO,
        "CameraTracking: square PID "
        "currentX=%.2f targetX=%d "
        "turn=%.4f "
        "right=%.2f left=%.2f",
        currentX,
        targetXCoordinate,
        turningPower,
        rightPower,
        leftPower);
  }

  // =====================================================
  // モータ出力
  // =====================================================

  robot
      .getWheelMotorControllerInstance()
      .setRightPower(
          rightPower);

  robot
      .getWheelMotorControllerInstance()
      .setLeftPower(
          leftPower);
}

void CameraTracking::wait()
{
  /*
   * 0msだとClockUtil::sleepで警告が出るため1ms。
   */
  ClockUtil::sleep(1);
}

void CameraTracking::finish()
{
  if(isStopMotorPower) {

    robot
        .getWheelMotorControllerInstance()
        .stopBoth();
  }

  Logger::info(
      "CameraTracking: finished");
}

double CameraTracking::getTargetSpeed() const
{
  return targetSpeed;
}

int CameraTracking::getTargetXCoordinate() const
{
  return targetXCoordinate;
}

const CameraServer::ColorRegionDetectorRequest&
CameraTracking::getColorDetectionRequest() const
{
  return colorDetectionRequest;
}

const CameraServer::QrCodeDetectorRequest&
CameraTracking::getQrDetectionRequest() const
{
  return qrDetectionRequest;
}

const CameraServer::SquareDetectorRequest&
CameraTracking::getSquareDetectionRequest() const
{
  return squareDetectionRequest;
}

CameraTracking::DetectionMode
CameraTracking::getDetectionMode() const
{
  return detectionMode;
}

bool CameraTracking::getIsStopMotorPower() const
{
  return isStopMotorPower;
}