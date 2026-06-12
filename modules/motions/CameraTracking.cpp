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
                               const CameraServer::ColorRegionDetectorRequest& _detectionRequest)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    pidGain(_pidGain),
    detectionRequest(_detectionRequest),
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

void CameraTracking::prepare() {}

void CameraTracking::executeStep()
{
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  SocketClient& client = robot.getCameraSocketClientInstance();
  CameraServer::ColorRegionDetectorResponse response;
  // run()の中でColorRegionDetectorインスタンスが繰り返し生死していた部分
  bool success = client.executeColorRegionDetection(detectionRequest, response);

  if(!success || !response.result.wasDetected) {
    Logger::warning("Color region not detected");
    return;
  }

  double currentX = (response.result.topLeft.x + response.result.bottomRight.x) / 2.0;
  double turningPower = cameraPid.calculatePid(currentX) * -1;
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
  robot.getWheelMotorControllerInstance().stopBoth();
}