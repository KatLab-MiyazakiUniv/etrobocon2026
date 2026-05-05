/**
 * @file WheelMotorController.h
 * @brief 左右のタイヤモータ制御に用いる関数をまとめたラッパークラス
 * @author sadomiya-sousi
 */
#ifndef WHEEL_MOTOR_CONTROLLER_H
#define WHEEL_MOTOR_CONTROLLER_H

#include "Motor.h"
// #include "SystemInfo.h"
#include "MotorSystemInfo.h"

using namespace spikeapi;

class WheelMotorController {
 public:
  /** Power値の上限 */
  static constexpr int MOTOR_POWER_MAX = 100;

  /** Power値の下限 */
  static constexpr int MOTOR_POWER_MIN = -100;

  /**
   * コンストラクタ
   */
  WheelMotorController();

  /**
   * @brief 右タイヤのモータにPower値をセット
   * @param power Power値
   */
  void setRightMotorPower(int power);

  /**
   * @brief 左タイヤのモータにPower値をセット
   * @param power Power値
   */
  void setLeftMotorPower(int power);

  /**
   * @brief 右タイヤのモータのpower値を0にリセット
   */
  void resetRightMotorPower();

  /**
   * @brief 左タイヤのモータのpower値を0にリセット
   */
  void resetLeftMotorPower();

  /**
   * @brief 両タイヤのモータのpower値を0にリセット
   */
  void resetWheelsMotorPower();

  /**
   * @brief 右タイヤのモータに, 線速度から算出した回転速度をセット
   * @param linearSpeedMmPerSec 線速度（mm/秒）
   */
  void setRightMotorSpeed(double linearSpeedMmPerSec);

  /**
   * @brief 左タイヤのモータに, 線速度から算出した回転速度をセット
   * @param linearSpeedMmPerSec 線速度（mm/秒）
   */
  void setLeftMotorSpeed(double linearSpeedMmPerSec);

  /**
   * @brief 両タイヤのモータを停止する
   */
  void stopWheelsMotor();

  /**
   * @brief ブレーキをかけてタイヤのモータを停止する
   */
  void brakeWheelsMotor();

  /**
   * @brief 右モータの角位置を取得する
   * @return 右モータの角位置（°）
   */
  int32_t getRightMotorCount();

  /**
   * @brief 左モータの角位置を取得する
   * @return 左モータの角位置（°）
   */
  int32_t getLeftMotorCount();

  /**
   * @brief 右タイヤのpower値を取得する
   * @return 右タイヤのpower値
   */
  int getRightMotorPower();

  /**
   * @brief 左タイヤのpower値を取得する
   * @return 左タイヤのpower値
   */
  int getLeftMotorPower();

  /**
   * @brief 右タイヤの線速度を取得する
   * @return 右タイヤの線速度（mm/秒）
   */
  double getRightMotorSpeed();

  /**
   * @brief 左タイヤの線速度を取得する
   * @return 左タイヤの線速度（mm/秒）
   */
  double getLeftMotorSpeed();

 private:
  spikeapi::Motor rightWheel;  // 右タイヤモータのインスタンス
  spikeapi::Motor leftWheel;   // 左タイヤモータのインスタンス

  /**
   * @brief モータに設定するpower値の制限
   * @param inputpower 入力されたpower値
   * @return 制限されたpower値
   */
  int limitPowerValue(int inputPower);
};

#endif
