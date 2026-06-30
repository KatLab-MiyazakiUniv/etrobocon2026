#include "Odometry.h"

#include <cmath>

#include "AngleNormalizer.h"
#include "Mileage.h"

Odometry::Odometry(Position& position)
  : position(position), prevLeft(0), prevRight(0), prevHeading(0.0)
{
}

void Odometry::reset()
{
  prevLeft = 0;
  prevRight = 0;
  prevHeading = 0.0;
  position.set(0.0, 0.0, 0.0);
}

void Odometry::update(int32_t left, int32_t right, double heading)
{
  // エンコーダ差分
  int32_t dLeft = left - prevLeft;
  int32_t dRight = right - prevRight;

  // 差分から移動距離を計算
  double distance = Mileage::calculateMileage(dRight, dLeft);

  // IMU角度を正規化
  heading = AngleNormalizer::normalizeAngle(heading);

  // 前回から今回までの角度差
  double deltaHeading = AngleNormalizer::normalizeAngle(heading - prevHeading);

  // 中点法
  double midHeading = AngleNormalizer::normalizeAngle(prevHeading + deltaHeading / 2.0);

  // ラジアンへ変換
  double rad = midHeading * DEG_TO_RAD;

  // 自己位置更新
  double newX = position.getX() + distance * std::cos(rad);
  double newY = position.getY() + distance * std::sin(rad);

  position.set(newX, newY, heading);

  // 次回用に保存
  prevLeft = left;
  prevRight = right;
  prevHeading = heading;
}