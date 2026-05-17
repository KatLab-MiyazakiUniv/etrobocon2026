/**
 * @file   Rotation.h
 * @brief  IMU方位角を用いたその場回転動作を実行するクラス
 * @author yutaro-1214
 */

#ifndef ROTATION_H
#define ROTATION_H

#include "BaseMotion.h"

class Rotation : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @brief Rotation を初期化する
   * @param _robot Robotクラスのインスタンスの参照
   * @param _continuationCondition 動作継続条件
   * @param _targetTurnPower 回転時のモータ出力
   */
  Rotation(Robot& _robot, std::unique_ptr<BaseContinuationCondition> _continuationCondition,
           float _targetTurnPower);

  /**
   * @brief 1周期分の動作を実行する
   */
  void executeStep() override;

  /**
   * @brief 動作終了時に両モータを停止する
   */
  void finish() override;

 private:
  float targetTurnPower;  // 回転時のモータ出力
};

#endif