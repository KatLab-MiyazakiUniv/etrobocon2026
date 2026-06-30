#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <cstdint>

#include "Position.h"
#include "SystemInfo.h"

class Odometry {
 public:
  /**
   * @brief コンストラクタ
   * @param position 更新対象
   */
  explicit Odometry(Position& position);

  /**
   * @brief エンコーダとIMUから自己位置を更新
   * @param left 左エンコーダ値[deg]
   * @param right 右エンコーダ値[deg]
   * @param heading IMU方位角[deg]
   */
  void update(int32_t left, int32_t right, double heading);

  /**
   * @brief エンコーダ値をリセット
   */
  void reset();

 private:
  Position& position;
  double prevHeading = 0.0;

  int32_t prevLeft;
  int32_t prevRight;
};

#endif