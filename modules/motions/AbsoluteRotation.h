/**
 * @file   AbsoluteRotation.h
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ABSOLUTE_ROTATION_H
#define ABSOLUTE_ROTATION_H

#include "Rotation.h"

/**
 * @class AbsoluteRotation
 * @brief 絶対角度を目標として回頭動作を行うクラス
 *
 * 指定された絶対角度に向けて、
 * PID制御を用いた回頭動作を実行する。
 */
class AbsoluteRotation : public Rotation {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot                 ロボット本体への参照
   * @param _continuationCondition 動作継続条件
   * @param _anglePidGain          回頭制御用PIDゲイン
   * @param _specifiedTargetAngle  目標となる絶対角度
   */
  AbsoluteRotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain, double _specifiedTargetAngle);

 protected:
  /**
   * @brief 回頭動作の準備処理を行う
   */
  void prepare() override;

 private:
  /**
   * @brief 目標となる絶対角度
   */
  double specifiedTargetAngle;
};

#endif
