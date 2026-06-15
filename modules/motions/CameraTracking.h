/**
 * @file   CameraTracking.h
 * @brief  カメラを使ったPID走行クラス
 * @author sadomiya-sousi
 */

#ifndef CAMERA_TRACKING_H
#define CAMERA_TRACKING_H

#include "BaseMotion.h"
#include "Pid.h"
#include "SpeedCalculator.h"
#include "SocketProtocol.h"

class CameraTracking : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief カメラ画像を使ったPID走行クラスを初期化する
   * @param _robot ロボットインスタンス
   * @param _continuationCondition 継続条件クラスのインスタンス
   * @param _targetSpeed 目標速度
   * @param _targetXCoordinate 目標x座標
   * @param _pidGain カメラ制御用PIDゲイン
   * @param _rightPid 右タイヤ速度制御用PIDゲイン
   * @param _leftPid 左タイヤ速度制御用PIDゲイン
   * @param _detectionRequest 検出リクエスト
   * @param _isStopMotorPower モーターを停止するかどうか
   */
  CameraTracking(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                 double _targetSpeed, int _targetXCoordinate, const Pid::PidGain& _pidGain,
                 const Pid::PidGain& _rightPid, const Pid::PidGain& _leftPid,
                 const CameraServer::ColorRegionDetectorRequest& _detectionRequest,
                 bool _isStopMotorPower = true);

  /**
   * デストラクタ
   */
  ~CameraTracking();

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
   * @brief 1周期分の待機を行う
   */
  void wait();

  /**
   * @brief 動作終了後の処理を行う
   */
  void finish() override;

 protected:
  double targetSpeed;                                         // 目標速度
  int targetXCoordinate;                                      // 目標X座標
  Pid::PidGain pidGain;                                       // カメラ用のPIDゲイン構造体
  CameraServer::ColorRegionDetectorRequest detectionRequest;  // 検出リクエスト
  bool isStopMotorPower;                                      // モーターを停止するかどうか
  SpeedCalculator speedCalculator;                            // 目標速度に対するモータパワー計算
  Pid cameraPid;                                              // カメラ画像x座標に対するPID制御
};

#endif