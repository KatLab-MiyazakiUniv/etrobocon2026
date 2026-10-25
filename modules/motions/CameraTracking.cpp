/**
 * @file   CameraTracking.cpp
 * @brief  カメラを使ったPID走行クラス
 * @author sadomiya-sousi HaruArima08
 */

#include "CameraTracking.h"

CameraTracking::CameraTracking(
    Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
    double _targetSpeed, int _targetXCoordinate, const Pid::PidGain& _pidGain,
    const CameraServer::ColorRegionDetectorRequest& _colorDetectionRequest, bool _isStopMotorPower, double _deadbandRate, double _maxoutRate)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(DetectionMode::COLOR_REGION),
    colorDetectionRequest(_colorDetectionRequest),
    qrDetectionRequest(),
    isStopMotorPower(_isStopMotorPower),
    speedCalculator(_robot, _targetSpeed),
    deadbandRate(_deadbandRate),
    maxoutRate(_maxoutRate),
    cameraPid(_pidGain.kp, _pidGain.ki, _pidGain.kd, _targetXCoordinate)
{
  LOG_CREATE("CameraTracking");
}

CameraTracking::CameraTracking(Robot& _robot,
                               std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                               double _targetSpeed, int _targetXCoordinate,
                               const Pid::PidGain& _pidGain,
                               const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
                               bool _isStopMotorPower, double _deadbandRate, double _maxoutRate)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    targetXCoordinate(_targetXCoordinate),
    detectionMode(DetectionMode::QR_CODE),
    colorDetectionRequest(),
    qrDetectionRequest(_qrDetectionRequest),
    isStopMotorPower(_isStopMotorPower),
    speedCalculator(_robot, _targetSpeed),
    deadbandRate(_deadbandRate),
    maxoutRate(_maxoutRate),
    cameraPid(_pidGain.kp, _pidGain.ki, _pidGain.kd, _targetXCoordinate)
{
  LOG_CREATE("QrCameraTracking");
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

  // マックスアウトの割合が0.0〜1.0の範囲外の場合は開始しない
  if(maxoutRate < 0.0 || maxoutRate > 1.0) {
    Logger::warning("マックスアウトの割合は0.0〜1.0の範囲で設定してください");
    return false;
  }

  // デッドバンドの割合がマックスアウトの割合を上回る場合は開始しない
  if(deadbandRate < 0.0 || deadbandRate > maxoutRate) {
    Logger::warning("デッドバンドの割合は0.0以上かつマックスアウトの割合以下で設定してください");
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
  // グローバル変数として使用する場合に伴って、変数を初期化する。Byソウシ(要削除)
  GLOBAL_logData = {};

  GLOBAL_logData.targetSpeed = targetSpeed;
  // 初期Speed値を計算
  double baseRightPower = speedCalculator.calculateRightMotorPower();
  double baseLeftPower = speedCalculator.calculateLeftMotorPower();

  double basePower = (std::abs(baseRightPower) + std::abs(baseLeftPower)) / 2.0;

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
      // グローバル変数として使用する場合に伴って、変数を初期化する。Byソウシ(要削除)
      CsvLogger::add(GLOBAL_logData);
      GLOBAL_logData = {};
      return;
    }
    // 黒を除く最大面積の色範囲取得>
    if(response.largestColorIndex != -1) {
      Logger::printfLog(Logger::DEBUG, "CameraTracking:最大色の検知失敗");
    }

    if(colorDetectionRequest.requireLargestColorIndex != -1 && response.largestColorIndex != 3
       && response.largestColorIndex != -1) {
      Logger::printfLog(Logger::DEBUG, "CameraTracking:検知した最大色の添字は[%d]",
                        response.largestColorIndex);
      robot.setIndexOfLabel(response.largestColorIndex);
    }

    // バウンディングボックスの中心X座標を計算
    currentX = (response.result.topLeft.x + response.result.bottomRight.x) / 2.0;
  }

  else if(detectionMode == DetectionMode::QR_CODE) {
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
      // グローバル変数として使用する場合に伴って、変数を初期化する。Byソウシ(要削除)
      CsvLogger::add(GLOBAL_logData);
      GLOBAL_logData = {};
    return;
  }

  if(!wasDetected) {
    Logger::warning("CameraTracking:検出対象が検出できませんでした");
      // グローバル変数として使用する場合に伴って、変数を初期化する。Byソウシ(要削除)
      CsvLogger::add(GLOBAL_logData);
      GLOBAL_logData = {};

    return;
  }

  // 色領域検出処理後のモーターのパワー値を取得する
  GLOBAL_logData.afterDetectRightMotorPower
      = robot.getWheelMotorControllerInstance().getRightPower();
  GLOBAL_logData.afterDetectLeftMotorPower = robot.getWheelMotorControllerInstance().getLeftPower();

  // 旋回値の計算
  double turningPower = cameraPid.calculatePid(currentX) * -1;
  GLOBAL_logData.turningPower = turningPower;

  // モータのPower値をセット（前進の時0を下回らないように，後進の時0を上回らないようにセット）
  // double rightPower = baseRightPower > 0.0 ? std::max(baseRightPower - turningPower, 0.0)
  //                                          : std::min(baseRightPower + turningPower, 0.0);
  // double leftPower = baseLeftPower > 0.0 ? std::max(baseLeftPower + turningPower, 0.0)



  // デッドバンドとマックスアウトを適用
  double deadbandPower = deadbandRate * basePower;
  double maxoutPower = maxoutRate * basePower;

  // デッドバンド: 閾値未満の旋回値を無視し、微小な揺れを防ぐ
  if(std::abs(turningPower) < deadbandPower) {
    turningPower = 0.0;
  } else {
    // マックスアウト: 閾値を超える旋回値を制限し、急激な旋回を防ぐ
    turningPower = std::min(std::max(turningPower, -maxoutPower), maxoutPower);
  }


  // 右モータのPower値をセット（前進時は0を下回らない、後進時は0を上回らない）
  double rightPower = baseRightPower - turningPower;
  if (baseRightPower > 0.0) {
      rightPower = std::max(rightPower, 0.0);
  } else {
      rightPower = std::min(rightPower, 0.0);
  }

  // 左モータのPower値をセット
  double leftPower = baseLeftPower + turningPower;
  if (baseLeftPower > 0.0) {
      leftPower = std::max(leftPower, 0.0);
  } else {
      leftPower = std::min(leftPower, 0.0);
  }


  //旋回量の大小関係が逆転したかどうかを判定
    double rightTurningPower = -turningPower;
    double leftTurningPower = turningPower;
    bool BiggerBeforeRightTurningPower = false;
    bool BiggerAfterRightTurningPower = false;

    if(rightTurningPower > leftTurningPower){
         BiggerBeforeRightTurningPower = true;
    }
    if(rightPower > leftPower){
         BiggerAfterRightTurningPower = true;
    }
    if (BiggerBeforeRightTurningPower !=  BiggerAfterRightTurningPower) {
    Logger::info("旋回量の大小関係が逆転");
    GLOBAL_logData.isReverseTurningPowerAndFinalPower = 100.0;
    }else {
    GLOBAL_logData.isReverseTurningPowerAndFinalPower = 0.0;
    }

  GLOBAL_logData.finalRightMotorPower = rightPower;
  GLOBAL_logData.finalLeftMotorPower = leftPower;
  CsvLogger::add(GLOBAL_logData);

  robot.getWheelMotorControllerInstance().setRightPower(rightPower);
  robot.getWheelMotorControllerInstance().setLeftPower(leftPower);
}

void CameraTracking::wait() {}

void CameraTracking::finish()
{
  if(isStopMotorPower) {
    // Logger::info("AHK:---------CameraTracking::finish()---------");
    // Logger::printfLog(Logger::DEBUG, "rightPower値は[%d]",
    //                   robot.getWheelMotorControllerInstance().getRightPower());
    // Logger::printfLog(Logger::DEBUG, "leftPower値は[%d]",
    //                   robot.getWheelMotorControllerInstance().getLeftPower());

    ClockUtil::sleep(10);
    // robot.getWheelMotorControllerInstance().brakeBoth();
    robot.getWheelMotorControllerInstance().stopBoth();

    // Logger::printfLog(Logger::DEBUG, "stopBoth()後の右power値は[%d]",
    //                   robot.getWheelMotorControllerInstance().getRightPower());
    // Logger::printfLog(Logger::DEBUG, "stopBoth()後の左power値は[%d]",
    //                   robot.getWheelMotorControllerInstance().getLeftPower());
    // Logger::info("AHK:---------CameraTracking::finish()---------");
  }

  GLOBAL_logData.brightness = 9999;
  CsvLogger::add(GLOBAL_logData);
  // CsvLogger::outputToFile();
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
