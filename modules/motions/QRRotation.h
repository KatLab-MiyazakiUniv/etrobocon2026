/**
 * @file   QRRotation.h
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef QR_ROTATION_H
#define QR_ROTATION_H

#include "Rotation.h"
#include "AngleNormalizer.h"
#include "Logger.h"
#include <algorithm>
#include <cmath>
#include "GatePosition.h"

class QRRotation : public Rotation {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボット本体への参照
   * @param _continuationCondition 動作の継続条件（終了判定）
   * @param _anglePidGain 角度制御用のPIDゲイン
   * @param _relativeTargetAngle 回頭したい相対角度(°)
   * @param _qrDetectionRequest QRコード検出リクエスト
   * @param _decryptRequired 暗号解読するか
   * @details
   * ロボット、継続条件、PIDゲイン、および相対角度を受け取り、
   * 相対回転動作の初期設定を行う
   */
  QRRotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
             const Pid::PidGain& _anglePidGain, double _relativeTargetAngle,
             CameraServer::QrCodeDetectorRequest& _qrDetectionRequest, bool _decryptRequired);

  /**
   * デストラクタ
   */
  ~QRRotation();

 protected:
  /**
   * @brief 判定開始前の準備処理
   * 現在角度に relativeAngle を加算し、targetAngle を設定する
   */
  void prepare() override;

  /**
   * @brief 回頭動作を1ステップ実行する
   */
  void executeStep() override;

 private:
  double relativeTargetAngle;                     // 回頭したい相対角度
  static constexpr double MIN_TURN_POWER = 50.0;  // 実機で静止摩擦に合わせて調整
  static constexpr double MAX_TURN_POWER = 60.0;
  Pid::PidGain anglePidGain;
  CameraServer::QrCodeDetectorRequest qrDetectionRequest;  // QRコード検出リクエスト
  bool decryptRequired = false;                            // 暗号解読するか
};

#endif