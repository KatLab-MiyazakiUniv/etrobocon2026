/** @file AngleNormalizer.cpp
 *  @brief 角度正規化クラス
 *  @author migaku2645
 */

#include "AngleNormalizer.h"

double AngleNormalizer::NormalizeAngle(double angle)
{
  // 角度の誤差を-180度から180度の範囲に正規化
  while(angle > 180.0) {
    angle -= 360.0;
  }
  while(angle < -180.0) {
    angle += 360.0;
  }
  return angle;
}