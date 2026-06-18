/**
 * @file   CameraTracking.cpp
 * @brief  カメラを使ったPID走行クラス
 * @author sadomiya-sousi
 */

#include "CameraTracking.h"

CameraTracking::CameraTracking(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                               double _targetSpeed, int _targetXCoordinate,
                               const Pid::PidGain& _pidGain, const Pid::PidGain& _rightPid,
                               const Pid::PidGain& _leftPid,
                               const CameraServer::ColorRegionDetectorRequest& _detectionRequest,
                               bool _isStopMotorPower)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate + CAM_MAX_WIDTH / 2),
    detectionRequest(_detectionRequest),
    isStopMotorPower(_isStopMotorPower),
    speedCalculator(_robot, _rightPid, _leftPid, _targetSpeed),
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
    return false;
  }
  return true;
}

void CameraTracking::executeStep()
{
  // 初期Speed値を計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  // 色領域検出処理の呼び出し。
  SocketClient& client = robot.getCameraSocketClientInstance();
  CameraServer::ColorRegionDetectorResponse response;
  // run()の中でColorRegionDetectorインスタンスが繰り返し生死。インスタンスの生死のlogが重い処理
  bool success = client.executeColorRegionDetection(detectionRequest, response);

  // 通信失敗、または検出できなかった場合は、出力を更新せずに終了する
  if(!success || !response.result.wasDetected) {
    Logger::warning("CameraTracking:色領域が検出されませんでした");
    return;
  }

  // バウンディングボックスの中心X座標を計算
  double currentX = (response.result.topLeft.x + response.result.bottomRight.x) / 2.0;

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
  ClockUtil::sleep(30);  // カメラの撮影FPSに合わせて30ミリ秒待機する
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

void CameraTracking::setTargetSpeed(double _targetSpeed)
{
  targetSpeed = _targetSpeed;
}

int CameraTracking::getTargetXCoordinate() const
{
  return targetXCoordinate;
}

void CameraTracking::setTargetXCoordinate(int _targetXCoordinate)
{
  targetXCoordinate = _targetXCoordinate + CAM_MAX_WIDTH / 2;
}


const CameraServer::ColorRegionDetectorRequest& CameraTracking::getDetectionRequest() const
{
  return detectionRequest;
}

void CameraTracking::setDetectionRequest(
    const CameraServer::ColorRegionDetectorRequest& _detectionRequest)
{
  detectionRequest = _detectionRequest;
}

bool CameraTracking::getIsStopMotorPower() const
{
  return isStopMotorPower;
}

void CameraTracking::setIsStopMotorPower(bool _isStopMotorPower)
{
  isStopMotorPower = _isStopMotorPower;
}

const SpeedCalculator& CameraTracking::getSpeedCalculator() const
{
  return speedCalculator;
}

const Pid& CameraTracking::getCameraPid() const
{
  return cameraPid;
}