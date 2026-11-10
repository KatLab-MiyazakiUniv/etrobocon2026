/**
 * @file   ArmMotorMotion.h
 * @brief  アームモータを指定角度まで回転させるクラス
 * @author HaruArima08
 */

#ifndef ARM_MOTOR_MOTION_H
#define ARM_MOTOR_MOTION_H

#include <cstdint>
#include <memory>

#include "ArmMotorController.h"
#include "BaseMotion.h"

class ArmMotorMotion : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot Robotクラスのインスタンス
   * @param _continuationCondition 継続条件（MotionTimeConditionを想定）
   * @param _power アームモータに設定する出力
   * @param _targetCount モータの目標回転量[deg]
   */
  ArmMotorMotion(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                 int _power, int32_t _targetCount);

 protected:
  /**
   * @brief アームモータの初期位置を取得して動作を開始する
   */
  void prepare() override;

  /**
   * @brief アームモータの角度を確認する
   */
  void executeStep() override;

  /**
   * @brief アームモータの動作を終了する
   */
  void finish() override;

 private:
  std::unique_ptr<ArmMotorController> armMotor;  // アームモータのインスタンス
  int power;                                    // アームモータの出力
  int32_t targetCount;                          // 目標回転量[deg]
  int32_t startCount = 0;                       // 動作開始時の角位置[deg]
  int32_t lastCount = 0;                        // 前回確認した角位置[deg]
  int startTime = 0;                            // 動作開始時刻[ms]
  int lastMovementTime = 0;                     // 最後に角位置が変化した時刻[ms]
  int lastLogTime = 0;                          // 最後にログを出力した時刻[ms]
  bool completed = false;                       // 角度到達または異常停止で完了したか
  bool reachedTarget = false;                   // 目標回転量に到達したか
  bool stalled = false;                         // 回転が停止したか
  bool wrongDirection = false;                  // 指定方向と逆に動いたか
};

#endif  // ARM_MOTOR_MOTION_H