/**
 * @file Navigator.h
 * @brief 目標地点までの距離と方位角を計算するクラス
 * @author yutaro-1214
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Position.h"

class Navigator {
 public:
  explicit Navigator(const Position& position);

  /**
   * @brief 目標地点までの距離を取得
   * @param goalX 目標X座標[mm]
   * @param goalY 目標Y座標[mm]
   * @return 距離[mm]
   */
  double calculateDistance(double goalX, double goalY) const;

  /**
   * @brief 目標地点への方位角を取得
   * @param goalX 目標X座標[mm]
   * @param goalY 目標Y座標[mm]
   * @return 方位角[deg]
   */
  double calculateHeading(double goalX, double goalY) const;

 private:
  const Position& position;
};

#endif