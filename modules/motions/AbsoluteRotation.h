/**
 * @file   AbsoluteRotation.h
 * @brief  絶対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef ABSOLUTE_ROTATION_H
#define ABSOLUTE_ROTATION_H

#include "Rotation.h"
#include "Logger.h"

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
                   const Pid::PidGain& _anglePidGain, double _targetAbsAngle);

  /**
   * デストラクタ
   */
  ~AbsoluteRotation();

 protected:
  /**
   * @brief 回頭動作を開始する前に、子条件の準備処理を行う
   */
  void prepare() override;

  /**
   * @brief 回頭動作を1ステップ実行する
   */
  void executeStep() override;

 private:
  static constexpr double MIN_TURN_POWER = 25.0;  // 実機で静止摩擦に合わせて調整
  static constexpr double MAX_TURN_POWER = 60.0;
  Pid::PidGain anglePidGain;
  double targetAbsAngle;  // 目標となる絶対角度(°)
};

#endif
