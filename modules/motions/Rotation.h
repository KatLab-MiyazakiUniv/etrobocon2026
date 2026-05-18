/**
 * @file   Rotation.h
 * @brief  回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ROTATION_H
#define ROTATION_H

#include "BaseMotion.h"
#include "Pid.h"

/**
 * @class Rotation
 * @brief PID制御による回頭動作を行うクラス
 *
 * IMUから取得した現在角度と目標角度の偏差を用いて、
 * 左右モータ出力を制御し、その場回転を実行する。
 */
class Rotation : public BaseMotion {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot                 ロボット本体への参照
   * @param _continuationCondition 動作継続条件
   * @param _anglePidGain          回頭制御用PIDゲイン
   */
  Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           const Pid::PidGain& _anglePidGain);

 protected:
  /**
   * @brief 回頭動作を1ステップ実行する
   */
  void executeStep() override;

  /**
   * @brief 回頭動作終了時の処理
   */
  void finish() override;

  /**
   * @brief 回頭制御で使用する目標角度
   */
  double targetAngle;

  /**
   * @brief 現在の機体角度を取得する
   * @return 現在の方位角
   */
  double getCurrentAngle();

  /**
   * @brief 角度を-180～180度の範囲に正規化する
   * @param angle 正規化前の角度
   * @return 正規化後の角度
   */
  double normalizeAngle(double angle);

 private:
  /**
   * @brief 回頭制御用PIDコントローラ
   */
  Pid anglePid;

  /**
   * @brief 現在の右モータ出力
   */
  double currentRightPower;

  /**
   * @brief 現在の左モータ出力
   */
  double currentLeftPower;
};

#endif