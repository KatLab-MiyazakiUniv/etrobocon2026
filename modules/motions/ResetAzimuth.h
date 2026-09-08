/**
 * @file ResetAzimuth.h
 * @brief 現在の機体方位を基準角度（0度）に設定する動作
 * @author miyahara046
 */
#ifndef RESET_AZIMUTH_H
#define RESET_AZIMUTH_H

#include "BaseMotion.h"
#include "Logger.h"

class ResetAzimuth : public BaseMotion {
 public:
  /**
   * コンストラクタ
   * @param robot ロボットクラスのインスタンス
   * @param continuationCondition 動作を継続する条件を判定するクラスのインスタンス
   */
  ResetAzimuth(Robot& robot, std::unique_ptr<BaseContinuationCondition> continuationCondition);
  /**
   * デストラクタ
   */
  ~ResetAzimuth();

 protected:
  /**
   * @brief 1周期分の動作として、現在の機体方位を基準角度（0度）に設定する
   */
  void executeStep() override;
};

#endif
