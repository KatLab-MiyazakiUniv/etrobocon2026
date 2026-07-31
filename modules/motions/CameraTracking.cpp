/**
 * @file   CameraTracking.cpp
 * @brief  カメラを使ったPID走行クラス
 * @author sadomiya-sousi HaruArima08
 */

#include "CameraTracking.h"

CameraTracking::CameraTracking(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                               double _targetSpeed, int _targetXCoordinate,
                               const Pid::PidGain& _pidGain,
                               const CameraServer::ColorRegionDetectorRequest& _detectionRequest,
                               bool _isStopMotorPower)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(DetectionMode::COLOR_REGION),
    detectionRequest(_detectionRequest),
    qrDetectionRequest(),
    isStopMotorPower(_isStopMotorPower),
    speedCalculator(_robot, _targetSpeed),
    cameraPid(_pidGain.kp, _pidGain.ki, _pidGain.kd, _targetXCoordinate)
{
  LOG_CREATE("CameraTracking");
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
    detectionRequest(),
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

  // 検出処理の呼び出し。
  SocketClient& client = robot.getCameraSocketClientInstance();
  bool success = false;
  bool wasDetected = false;
  double currentX = 0.0;

  if(detectionMode == DetectionMode::COLOR_REGION) {
    CameraServer::ColorRegionDetectorResponse response;
    // run()の中でColorRegionDetectorインスタンスが繰り返し生死。インスタンスの生死のlogが重い処理
    success = client.executeColorRegionDetection(detectionRequest, response);
    wasDetected = response.result.wasDetected;
    if(success && wasDetected) {
      // バウンディングボックスの中心X座標を計算
      currentX = (response.result.topLeft.x + response.result.bottomRight.x) / 2.0;
    }
  } else {
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
  }

  // 通信失敗、または検出できなかった場合は、出力を更新せずに終了する
  if(!success || !wasDetected) {
    Logger::warning("CameraTracking:検出対象が検出されませんでした");
    return;
  }

  // 旋回値の計算
  double turningPower = cameraPid.calculatePid(currentX) * -1;

  // モータのPower値をセット（前進の時0を下回らないように，後進の時0を上回らないようにセット）
  double rightPower = baseRightPower > 0.0 ? std::max(baseRightPower - turningPower, 0.0)
                                           : std::min(baseRightPower + turningPower, 0.0);
  double leftPower = baseLeftPower > 0.0 ? std::max(baseLeftPower + turningPower, 0.0)
                                         : std::min(baseLeftPower - turningPower, 0.0);

  robot.getWheelMotorControllerInstance().setRightPower(rightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(leftPower);
}

void CameraTracking::wait()
{
  ClockUtil::sleep(0);
}

void CameraTracking::finish()
{
  if(isStopMotorPower) {
    robot.getWheelMotorControllerInstance().stopBoth();
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

const CameraServer::ColorRegionDetectorRequest& CameraTracking::getDetectionRequest() const
{
  return detectionRequest;
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