/**
 * @file   Odometry.h
 * @brief  移動距離と向いている方向からPositionを更新するクラス
 * @author yutaro-1214
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Position.h"

class Odometry {
 public:
  /**
   * @brief コンストラクタ
   * @param position 更新対象の位置情報
   */
  explicit Odometry(Position& position);

  /**
   * @brief 位置を更新
   * @param distance 移動距離[mm]
   * @param heading IMU方位角[deg]
   */
  void update(double distance, double heading);

 private:
  Position& position;
};

#endif