/** @file AngleNormalizer.h
 *  @brief 角度正規化クラス
 *  @author migaku2645
 */

#ifndef ANGLE_NORMALIZER_H
#define ANGLE_NORMALIZER_H

class AngleNormalizer {
 public:
  /**
   * @brief 角度を-180度から180度の範囲に正規化する
   * @param angle 正規化する角度(°)
   * @return 正規化された角度(°)
   */
  static double normalizeAngle(double angle);

 private:
  AngleNormalizer();
};

#endif  // ANGLE_NORMALIZER_H