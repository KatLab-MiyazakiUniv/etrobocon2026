
/**
 * @file   IMUCalibrator.h
 * @brief  IMUの静止時角速度を測定し、バイアスを補正するクラス
 * @author yutaro-1214
 */

#ifndef IMU_CALIBRATOR_H
#define IMU_CALIBRATOR_H

#include "IMUController.h"

class IMUCalibrator {
 public:
  /**
   * @brief コンストラクタ
   * @param imuController IMU制御クラス
   */
  explicit IMUCalibrator(IMUController& imuController);

  /**
   * @brief キャリブレーションを開始する
   * @param sampleCount 必要なサンプル数
   */
  void start(unsigned int sampleCount = 500);

  /**
   * @brief 静止中の角速度を1回測定する
   * @return true 測定完了、false 測定中または測定失敗
   */
  bool update();

  /**
   * @brief 補正済みの角速度を取得する
   * @param ang 補正済みの角速度
   */
  void getCorrectedAngularVelocity(IMUController::AngularVelocity& ang);

  /**
   * @brief 測定したバイアスを取得する
   * @return 各軸の平均角速度
   */
  IMUController::AngularVelocity getBias() const;

  /**
   * @brief 測定した標準偏差を取得する
   * @return 各軸の標準偏差
   */
  IMUController::AngularVelocity getStandardDeviation() const;

  /**
   * @brief キャリブレーション完了判定
   * @return true 完了、false 未完了
   */
  bool isCalibrated() const;

  /**
   * @brief 現在のサンプル数を取得する
   * @return サンプル数
   */
  unsigned int getSampleCount() const;

 private:
  IMUController& imuController;

  unsigned int requiredSamples = 0;
  unsigned int currentSamples = 0;

  bool calibrated = false;
  bool calibrating = false;

  IMUController::AngularVelocity bias{};
  IMUController::AngularVelocity standardDeviation{};

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  double sumSquareX = 0.0;
  double sumSquareY = 0.0;
  double sumSquareZ = 0.0;
};

#endif  // IMU_CALIBRATOR_H
