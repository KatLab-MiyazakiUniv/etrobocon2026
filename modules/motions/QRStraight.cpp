/**
 * @file   QRStraight.cpp
 * @brief  QR検出をしながら直進動作を実行するクラス
 * @author migaku2645
 */

#include "QRStraight.h"

QRStraight::QRStraight(Robot& _robot,
                       std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                       double _targetSpeed, const Pid::PidGain& _pidGain, bool _shouldUseIMU,
                       double _deadbandRate, double _maxoutRate,
                       const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest,
                       bool _decryptRequired)
  : BaseMotion(_robot, std::move(_continuationCondition)),
    targetSpeed(_targetSpeed),
    speedCalculator(_robot, _targetSpeed),
    qrDetectionRequest(_qrDetectionRequest),
    decryptRequired(_decryptRequired),
    anglePid(_anglePidGain.kp, _anglePidGain.ki, _anglePidGain.kd, 0.0),
    shouldUseIMU(_shouldUseIMU),
    targetAngle(0.0),
    deadbandRate(_deadbandRate),
    maxoutRate(_maxoutRate)
{
  LOG_CREATE("QRStraight");
}

QRStraight::~QRStraight()
{
  LOG_DESTROY("QRStraight");
}

bool QRStraight::canStart()
{
  // targetSpeed値が0の場合は終了する
  if(targetSpeed == 0.0) {
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

void QRStraight::prepare()
{
  // IMU有りの場合、走行前の角度を目標角度に設定
  if(shouldUseIMU) {
    targetAngle = robot.getIMUControllerInstance().getAzimuth();
  }

  anglePid.prepare();
}
void QRStraight::executeStep()
{
  // QR検出処理

  // QR検出済みなら何もしない
  if(decryptRequired && !robot.getHaveQR2contents()
     || !decryptRequired && !robot.getHaveQR1contents()) {
    // QRコードを検出

    SocketClient& client = robot.getCameraSocketClientInstance();
    bool success = false;
    bool wasDetected = false;

    CameraServer::QrCodeDetectorResponse qrResponse;
    success = client.executeQrCodeDetection(qrDetectionRequest, qrResponse);
    wasDetected = qrResponse.wasDetected;

    if(!wasDetected || !success) {
      Logger::error("GatePositionDetection: QRコードの検出に失敗しました。");
      return;
    } else {
      Logger::error("GatePositionDetection: QRコードの検出に成功しました。");

      // QR検出成功ならフラグを更新
      if(decryptRequired) {
        robot.setHaveQR2contents();
      } else {
        robot.setHaveQR1contents();
        //  QRコードの暗号文を復号
        std::string gatePositionData;

        if(decryptRequired) {
          // AesDecryptor decryptor(std::string(robot.getDecryptionKey()), qrResponse.content);
          // 青・黄色：復号する
          gatePositionData
              = Decrypter::decrypt(std::string(robot.getDecryptionKey()), qrResponse.content);

          if(gatePositionData.empty()) {
            Logger::error("GatePositionDetection: QRコードの復号に失敗しました。");
            return;
          }
        } else {
          // 赤：復号しない
          gatePositionData = qrResponse.content;
        }

        //  平文を解析して,ゲート位置情報をRobotに設定

        GatePositionParser parser(gatePositionData, robot);

        if(!parser.parse()) {
          Logger::error("GatePositionDetection: ゲート位置情報の解析に失敗しました。");
          return;
        }

        Logger::info("GatePositionDetection: ゲート位置情報の設定に成功しました。");
      }
    }
  }

  double requiredRightPower = speedCalculator.calculateRightMotorPower();
  double requiredLeftPower = speedCalculator.calculateLeftMotorPower();
  double turningPower = 0.0;

  if(shouldUseIMU) {
    // 現在角度を取得
    double currentAngle = robot.getIMUControllerInstance().getAzimuth();

    // 目標角度との差を計算
    double angleDeviation = targetAngle - currentAngle;

    // -180〜180度に正規化
    angleDeviation = AngleNormalizer::normalizeAngle(angleDeviation);

    // PID制御で旋回量を計算
    turningPower = anglePid.calculatePid(angleDeviation);

    double basepower = (std::abs(requiredRightPower) + std::abs(requiredLeftPower)) / 2.0;

    // デッドバンドとマックスアウトを計算
    double deadbandPower = deadbandRate * basepower;
    double maxoutPower = maxoutRate * basepower;

    // デッドバンド
    if(std::abs(turningPower) < deadbandPower) {
      turningPower = 0.0;
    } else {
      // マックスアウト
      turningPower = std::min(std::max(turningPower, -maxoutPower), maxoutPower);
    }
  }
  // モーターにPower値をセット
  robot.getWheelMotorControllerInstance().setRightPower(requiredRightPower + turningPower);
  robot.getWheelMotorControllerInstance().setLeftPower(requiredLeftPower - turningPower);
}
void QRStraight::finish()
{
  ClockUtil::sleep(10);
  robot.getWheelMotorControllerInstance().stopBoth();
  Logger::info("GatePositionDetection: ゲート位置情報の設定に成功しました。")
}
