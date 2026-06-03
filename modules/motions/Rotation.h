/**
 * @file   Rotation.h
 * @brief  回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ROTATION_H
#define ROTATION_H

#include "BaseMotion.h"
#include "Pid.h"
#include "AngleNormalizer.h"

class Rotation : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot                 ロボットクラスのインスタンス
   * @param _continuationCondition 動作継続条件を判定するクラスのインスタンス
   * @param _anglePidGain          回頭制御用PIDゲイン
   */
  Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           const Pid::PidGain& _anglePidGain);

 protected:
  double targetAngle;  // 回頭制御で使用する目標角度(°)

  /**
   * @brief 回頭動作を1ステップ実行する
   */
  void executeStep() override;

  /**
   * @brief 回頭動作終了時の処理
   */
  void finish() override;

  /**
   * @brief 現在の方位角を取得する
   * @return 現在の方位角(°)
   */
  double getCurrentAngle();

 private:
  Pid anglePid;  // 回頭制御用PIDコントローラ
};

#endif