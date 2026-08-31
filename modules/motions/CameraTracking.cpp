/**
 * @file   CameraTracking.cpp
 * @brief  カメラを使ったPID走行クラス
 * @author sadomiya-sousi HaruArima08
 */

#include "CameraTracking.h"

CameraTracking::CameraTracking(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    double _targetSpeed, int _targetXCoordinate, const Pid::PidGain& _pidGain,
    const CameraServer::ColorRegionDetectorRequest& _colorDetectionRequest, bool _isStopMotorPower)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(DetectionMode::COLOR_REGION),
    colorDetectionRequest(_colorDetectionRequest),
    qrDetectionRequest(),
    isStopMotorPower(_isStopMotorPower),
    speedCalculator(_robot, _targetSpeed),
    cameraPid(_pidGain.kp, _pidGain.ki, _pidGain.kd, _targetXCoordinate)
{
  LOG_CREATE("CameraTracking");
  Logger::printfLog(Logger::DEBUG, "カメラトラッキング作成");
}

CameraTracking::CameraTracking(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                               double _targetSpeed, int _targetXCoordinate,
                               const Pid::PidGain& _pidGain,
                               const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
                               bool _isStopMotorPower)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(DetectionMode::QR_CODE),
    colorDetectionRequest(),
    qrDetectionRequest(_qrDetectionRequest),
    isStopMotorPower(_isStopMotorPower),
    speedCalculator(_robot, _targetSpeed),
    cameraPid(_pidGain.kp, _pidGain.ki, _pidGain.kd, _targetXCoordinate)
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
    Logger::error("CameraTracking:目標スピードが0です");
    return false;
  }
  return true;
}

void CameraTracking::prepare()
{
  cameraPid.prepare();
}

void CameraTracking::executeStep()
{
  // 初期Speed値を計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  // 検出処理の呼び出し
  SocketClient& client = robot.getCameraSocketClientInstance();
  bool success = false;
  bool wasDetected = false;
  double currentX = 0.0;

  if(detectionMode == DetectionMode::COLOR_REGION) {
    CameraServer::ColorRegionDetectorResponse response;
    success = client.executeColorRegionDetection(colorDetectionRequest, response);
    wasDetected = response.result.wasDetected;

    if(!success || !wasDetected) {
      Logger::printfLog(Logger::WARNING,
                        "CameraTracking:色領域が検出されませんでした。success:%d wasDetected:%d",
                        success, wasDetected);
      return;
    }

    // 黒を除く最大面積の色範囲取得>

    if(response.largestColorIndex != -1) {
      Logger::printfLog(Logger::DEBUG, "CameraTracking:最大色の検知失敗");
    }

    if(colorDetectionRequest.requireLargestColorIndex != -1 && response.largestColorIndex != 3  && response.largestColorIndex != -1) {
      Logger::printfLog(Logger::DEBUG, "CameraTracking:検知した最大色の添字は[%d]", response.largestColorIndex);
      robot.setIndexOfLabel(response.largestColorIndex);
    }



    // バウンディングボックスの中心X座標を計算
    currentX = (response.result.topLeft.x + response.result.bottomRight.x) / 2.0;
  } else if(detectionMode == DetectionMode::QR_CODE) {
    CameraServer::QrCodeDetectorResponse response;
    success = client.executeQrCodeDetection(qrDetectionRequest, response);
    wasDetected = response.wasDetected;
    if(success && wasDetected) {
      // QRコードの4頂点の中心X座標を計算
      double sumX = 0.0;
      for(const auto& corner : response.corners) {
        sumX += corner.x;
      }
      currentX = sumX / CameraServer::QR_CODE_CORNER_COUNT;
    }
  } else {
    Logger::error("CameraTracking:検出方式が不正です");
  }

  if(!success) {
    Logger::warning("CameraTracking:通信に失敗しました");
    return;
  }

  if(!wasDetected) {
    Logger::warning("CameraTracking:検出対象が検出できませんでした");
    return;
  }

  // 旋回値の計算
  double turningPower = cameraPid.calculatePid(currentX) * -1;

  // モータのPower値をセット（前進の時0を下回らないように，後進の時0を上回らないようにセット）
  double rightPower = baseRightPower > 0.0 ? std::max(baseRightPower - turningPower, 0.0)
                                           : std::min(baseRightPower + turningPower, 0.0);
  double leftPower = baseLeftPower > 0.0 ? std::max(baseLeftPower + turningPower, 0.0)
                                         : std::min(baseLeftPower - turningPower, 0.0);

  Logger::printfLog(Logger::DEBUG, "カメラトラッキングのpower値をセットする直前");
  robot.getWheelMotorControllerInstance().setRightPower(rightPower);
  Logger::printfLog(Logger::DEBUG, "カメラトラッキングの右のpower値をセットしました");
  robot.getWheelMotorControllerInstance().setLeftPower(leftPower);
  Logger::printfLog(Logger::DEBUG, "カメラトラッキングの左のpower値をセットしました");
}

void CameraTracking::wait() {}

void CameraTracking::finish()
{
  if(isStopMotorPower) {
    Logger::printfLog(Logger::DEBUG, "モーターに0をセット");
    // robot.getWheelMotorControllerInstance().stopBoth();
    robot.getWheelMotorControllerInstance().brakeBoth();
  }
}

double CameraTracking::getTargetSpeed() const
{
  return targetSpeed;
}

int CameraTracking::getTargetXCoordinate() const
{
  return targetXCoordinate;
}

const CameraServer::ColorRegionDetectorRequest& CameraTracking::getColorDetectionRequest() const
{
  return colorDetectionRequest;
}

const CameraServer::QrCodeDetectorRequest& CameraTracking::getQrDetectionRequest() const
{
  return qrDetectionRequest;
}

CameraTracking::DetectionMode CameraTracking::getDetectionMode() const
{
  return detectionMode;
}

bool CameraTracking::getIsStopMotorPower() const
{
  return isStopMotorPower;
}