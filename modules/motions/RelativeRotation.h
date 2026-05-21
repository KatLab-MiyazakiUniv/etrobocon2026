/**
 * @file   RelativeRotation.h
 * @brief  相対角度で回頭動作を実行するクラス
 * @author okuyama0528 yutaro-1214
 */
#ifndef RELATIVE_ROTATION_H
#define RELATIVE_ROTATION_H

#include "Rotation.h"

class RelativeRotation : public Rotation {
 public:
  /**
   * @brief コンストラクタ
   * @param _robot ロボット本体への参照
   * @param _continuationCondition 動作の継続条件（終了判定）
   * @param _anglePidGain 角度制御用のPIDゲイン
   * @param _relativeAngle 回転したい相対角度
   * @details
   * ロボット、継続条件、PIDゲイン、および相対角度を受け取り、
   * 相対回転動作の初期設定を行う
   */
  RelativeRotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
                   const Pid::PidGain& _anglePidGain, double _relativeAngle);

 protected:
  void prepare() override;  // 現在角度にrelativeAngleを加算し、目標角度を算出する

 private:
  double relativeAngle;  // 回頭したい相対角度
};

#endif