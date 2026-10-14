/**
 * @file   QRStraight.h
 * @brief  QR検出を行いながら直進するクラス
 * @author migaku2645
 */

#ifndef QR_STRAIGHT_H
#define QR_STRAIGHT_H

#include "BaseMotion.h"
#include "Pid.h"
#include "SpeedCalculator.h"
#include "SocketProtocol.h"
#include "AngleNormalizer.h"
#include "Logger.h"
#include <string>
#include "GatePositionDetection.h"
#include "RepeatCountCondition.h"

class QRStraight : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief QR検出を行いながら直進するクラスを初期化する
   * @param _robot ロボットインスタンス
   * @param _continuationCondition 継続条件クラスのインスタンス
   * @param _targetSpeed 目標速度
   * @param _anglePidGain 角度制御用PIDゲイン
   * @param _shouldUseIMU IMUを使用するかどうか
   * @param _deadbandRate turningPowerに適用するデッドバンドの割合(0.0〜1.0、basePowerに対する割合)
   * @param _maxoutRate turningPowerに適用するマックスアウトの割合(0.0〜1.0、basePowerに対する割合)
   * @param _decryptRequired 暗号解読するか
   * @param _qrDetectionRequest QRコード検出リクエスト
   */
  QRStraight(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
             double _targetSpeed, const Pid::PidGain& _anglePidGain, bool _shouldUseIMU,
             double _deadbandRate, double _maxoutRate,
             const CameraServer::QrCodeDetectorRequest& _qrDetectionRequest, bool _decryptRequired);

  /**
   * デストラクタ
   */
  ~QRStraight();

 protected:
  /**
   * @brief 動作を開始できるかどうかを判定する
   */
  bool canStart() override;

  /**
   * @brief 動作を開始する前に必要な準備を行う
   */
  void prepare() override;

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 動作終了後の処理を行う
   */
  void finish() override;

 private:
  double targetSpeed;               // 目標速度(mm/秒)
  double targetAngle;               // 目標角度(度)
  SpeedCalculator speedCalculator;  // SpeedCalculatorクラスのインスタンス
  Pid anglePid;                     // 角度制御用PID
  bool shouldUseIMU;                // IMUを使用するかどうか
  double deadbandRate;              // turningPowerに適用するデッドバンドの割合(0.0〜1.0)
  double maxoutRate;                // turningPowerに適用するマックスアウトの割合(0.0〜1.0)
  CameraServer::QrCodeDetectorRequest qrDetectionRequest;  // QRコード検出リクエスト
  bool decryptRequired = false;                            // 暗号解読するか
};
#endif
