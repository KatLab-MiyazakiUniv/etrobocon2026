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
   * @param _detectionRequest 検出リクエスト
   * @param _isStopMotorPower モーターを停止するかどうか
   */
  CameraTracking(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                 double _targetSpeed, int _targetXCoordinate, const Pid::PidGain& _pidGain,
                 const CameraServer::ColorRegionDetectorRequest& _detectionRequest,
                 bool _isStopMotorPower = true);

  /**
   * デストラクタ
   */
  ~CameraTracking();

  /**
   * @brief 目標速度を取得する
   * @return double 目標速度
   */
  double getTargetSpeed() const;

  /**
   * @brief 目標X座標を取得する
   * @return int 目標X座標
   */
  int getTargetXCoordinate() const;

  /**
   * @brief 色領域検出リクエストを取得する
   * @return const CameraServer::ColorRegionDetectorRequest& 検出リクエストへの参照
   */
  const CameraServer::ColorRegionDetectorRequest& getDetectionRequest() const;

  /**
   * @brief 色領域検出リクエストを設定する
   * @param _detectionRequest 設定する検出リクエスト
   */
  void setDetectionRequest(const CameraServer::ColorRegionDetectorRequest& _detectionRequest);

  /**
   * @brief モーターを停止するかどうかを取得する
   * @return true 停止する場合
   * @return false 停止しない場合
   */
  bool getIsStopMotorPower() const;

  /**
   * @brief 目標速度に対するモータパワー計算器を取得する
   * @return const SpeedCalculator& モータパワー計算器への参照
   */
  const SpeedCalculator& getSpeedCalculator() const;

  /**
   * @brief カメラ画像x座標に対するPID制御器を取得する
   * @return const Pid& PID制御器への参照
   */
  const Pid& getCameraPid() const;

 protected:
  /**
   * @brief 動作を開始できるかどうかを判定する
   */
  bool canStart() override;

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 1周期分の待機を行う
   */
  void wait() override;

  /**
   * @brief 動作終了後の処理を行う
   */
  void finish() override;

 private:
  double targetSpeed;                                         // 目標速度
  int targetXCoordinate;                                      // 目標X座標
  CameraServer::ColorRegionDetectorRequest detectionRequest;  // 検出リクエスト
  bool isStopMotorPower;                                      // モーターを停止するかどうか
  SpeedCalculator speedCalculator;                            // 目標速度に対するモータパワー計算
  Pid cameraPid;                                              // カメラ画像x座標に対するPID制御
};

#endif