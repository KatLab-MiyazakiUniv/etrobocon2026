/**
 * @file WheelMotorController.h
 * @brief タイヤモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */
#ifndef WHEEL_MOTOR_CONTROLLER_H
#define WHEEL_MOTOR_CONTROLLER_H

// #include "Motor.h"
#include "BaseMotorController.h"

using namespace spikeapi;

class WheelMotorController : public BaseMotorController {
 public:
  /**
   * コンストラクタ
   */
  WheelMotorController();

  /**
   * @brief 右タイヤモータにPower値をセット
   * @param power Power値
   */
  void setRightPower(int power);

  /**
   * @brief 左タイヤモータにPower値をセット
   * @param power Power値
   */
  void setLeftPower(int power);

  /**
   * @brief 右タイヤモータのpower値を0にリセット
   */
  void resetRightPower();

  /**
   * @brief 左タイヤモータのpower値を0にリセット
   */
  void resetLeftPower();

  /**
   * @brief 両タイヤモータのpower値を0にリセット
   */
  void resetBothPower();

  /**
   * @brief 右タイヤモータに, 線速度から算出した回転速度をセット
   * @param linearSpeedMmPerSec 線速度（mm/秒）
   */
  void setRightSpeed(double linearSpeedMmPerSec);

  /**
   * @brief 左タイヤモータに, 線速度から算出した回転速度をセット
   * @param linearSpeedMmPerSec 線速度（mm/秒）
   */
  void setLeftSpeed(double linearSpeedMmPerSec);

  /**
   * @brief 両タイヤモータを停止する
   */
  void stopBoth();

  /**
   * @brief ブレーキをかけてタイヤのモータを停止する
   */
  void brakeBoth();

  /**
   * @brief 右タイヤモータの角位置を取得する
   * @return 右タイヤモータの角位置（°）
   */
  int32_t getRightCount();

  /**
   * @brief 左タイヤモータの角位置を取得する
   * @return 左タイヤモータの角位置（°）
   */
  int32_t getLeftCount();

  /**
   * @brief 右タイヤモータのpower値を取得する
   * @return 右タイヤモータのpower値
   */
  int getRightPower();

  /**
   * @brief 左タイヤモータのpower値を取得する
   * @return 左タイヤモータのpower値
   */
  int getLeftPower();

  /**
   * @brief 右タイヤモータの線速度を取得する
   * @return 右タイヤモータの線速度（mm/秒）
   */
  double getRightSpeed();

  /**
   * @brief 左タイヤモータの線速度を取得する
   * @return 左タイヤモータの線速度（mm/秒）
   */
  double getLeftSpeed();

 private:
  spikeapi::Motor rightWheel;  // 右タイヤモータのインスタンス
  spikeapi::Motor leftWheel;   // 左タイヤモータのインスタンス
};

#endif
