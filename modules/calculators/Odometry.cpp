/**
 * @file   Odometry.cpp
 * @brief  移動距離と向いている方向からPositionを更新するクラス
 * @author yutaro-1214
 */

#include "Odometry.h"

#include <cmath>

namespace {
  constexpr double DEG_TO_RAD = M_PI / 180.0;  // 度からラジアンへ変換
}

Odometry::Odometry(Position& position) : position(position) {}

void Odometry::update(double distance, double heading)
{
  double rad = heading * DEG_TO_RAD;

  double newX = position.getX() + distance * std::cos(rad);

  double newY = position.getY() + distance * std::sin(rad);

  position.set(newX, newY, heading);
}