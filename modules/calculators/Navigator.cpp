/**
 * @file Navigator.cpp
 * @brief 目標地点までの距離と方位角を計算するクラス
 * @author yutaro-1214
 */

#include "Navigator.h"

#include <cmath>

namespace {
  constexpr double RAD_TO_DEG = 180.0 / M_PI;
}

Navigator::Navigator(const Position& position) : position(position) {}

double Navigator::calculateDistance(double goalX, double goalY) const
{
  double dx = goalX - position.getX();
  double dy = goalY - position.getY();

  return std::sqrt(dx * dx + dy * dy);
}

double Navigator::calculateHeading(double goalX, double goalY) const
{
  double dx = goalX - position.getX();
  double dy = goalY - position.getY();

  double heading = std::atan2(dy, dx) * RAD_TO_DEG;

  return AngleNormalizer::normalizeAngle(heading);
}