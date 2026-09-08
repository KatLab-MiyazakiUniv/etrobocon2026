/**
 * @file ProjectedMileage.cpp
 * @brief 基準方向への符号付き移動距離を計算するクラス
 * @author miyahara046
 */

#include "ProjectedMileage.h"
#include "AngleNormalizer.h"
#include "SystemInfo.h"
#include <cmath>
#include "Logger.h"

void ProjectedMileage::reset(double mileage, double angle)
{
  // NaNや無限大を使うと以降の距離判定が成立しなくなるため、計測を無効にする。
  valid = std::isfinite(mileage) && std::isfinite(angle);
  if(!valid) {
    Logger::error("ProjectedMileage: 起点の距離または角度が有限値ではありません");
    return;
  }
  // 車輪の累計値は変更せず、今回の計測で使う起点だけを記録する。
  previousMileage = mileage;
  previousAngle = angle;
  distance = 0.0;
}

void ProjectedMileage::update(double mileage, double angle)
{
  if(!valid) {
    Logger::error("ProjectedMileage: 距離計測が無効です。resetで起点を設定してください");
    return;
  }
  if(!std::isfinite(mileage) || !std::isfinite(angle)) {
    valid = false;
    Logger::error("ProjectedMileage: 更新時の距離または角度が有限値ではありません");
    return;
  }
  // ±180度の境界を考慮し、区間の中央方位で微小移動を基準方向に換算する。
  double midpoint = previousAngle + AngleNormalizer::normalizeAngle(angle - previousAngle) / 2.0;
  // 差分距離 × cos(方位)を積算する。後退は負の差分になるため減算される。
  double nextDistance = distance + (mileage - previousMileage) * std::cos(midpoint * PI / 180.0);
  if(!std::isfinite(nextDistance)) {
    valid = false;
    Logger::error("ProjectedMileage: 基準方向の距離計算で有限値の範囲を超えました");
    return;
  }
  distance = nextDistance;
  // 次の周期では、今回の測定値との差分を使う。
  previousMileage = mileage;
  previousAngle = angle;
}

double ProjectedMileage::getDistance() const
{
  return distance;
}

bool ProjectedMileage::isValid() const
{
  return valid;
}
