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
  ResetAzimuth(Robot& robot, std::unique_ptr<BaseContinuationCondition> continuationCondition);
  ~ResetAzimuth();

 protected:
  void executeStep() override;
};

#endif
