/**
 * @file   AbsoluteRotation.h
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ABSOLUTE_ROTATION_H
#define ABSOLUTE_ROTATION_H

#include "Rotation.h"

class AbsoluteRotation : public Rotation {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot                 ロボットクラスのインスタンス
   * @param _continuationCondition 動作継続条件を判定するクラスのインスタンス
   * @param _anglePidGain          回頭制御用PIDゲイン
   * @param _targetAbsAngle  目標となる絶対角度(°)
   */
  AbsoluteRotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain, double _targetAbsAngle, double _basePower);

 protected:
  // 回頭動作の準備処理を行う
  void prepare() override;

 private:
  double targetAbsAngle;  // 目標となる絶対角度(°)
};

#endif
