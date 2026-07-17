/**
 * @file   Odometry.cpp
 * @brief  移動距離と向いている方向からPositionを更新するクラス
 * @author yutaro-1214
 */

#include "Odometry.h"
#include <cmath>
#include "AngleNormalizer.h"
#include "Mileage.h"

Odometry::Odometry(Position& position) : position(position), prevLeft(0), prevRight(0) {}

void Odometry::reset()
{
  // Positionをリセット
  position.set(0.0, 0.0, 0.0);
}

void Odometry::initialize(int32_t left, int32_t right)
{
  // エンコーダ値をセット
  prevLeft = left;
  prevRight = right;
}

void Odometry::update(int32_t left, int32_t right, double heading)
{
  // エンコーダ差分
  int32_t dLeft = left - prevLeft;
  int32_t dRight = right - prevRight;

  // 差分から移動距離を計算
  double distance = Mileage::calculateMileage(dRight, dLeft);

  // 現在値を保存
  prevLeft = left;
  prevRight = right;

  // IMU角度を正規化
  heading = AngleNormalizer::normalizeAngle(heading);

  double rad = heading * DEG_TO_RAD;

  // 自己位置更新
  double newX = position.getX() + distance * std::cos(rad);
  double newY = position.getY() + distance * std::sin(rad);

  position.set(newX, newY, heading);
}