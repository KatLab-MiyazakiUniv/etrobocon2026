/**
 * @file ProjectedMileage.h
 * @brief 基準方向への符号付き移動距離を計算するクラス
 * @author miyahara046
 */

#ifndef PROJECTED_MILEAGE_H
#define PROJECTED_MILEAGE_H

class ProjectedMileage {
 public:
  /**
   * @brief 距離計測の起点を設定する
   * @param mileage 車輪から求めた累計走行距離[mm]
   * @param angle 基準を0度とした現在方位[deg]
   */
  void reset(double mileage, double angle);

  /**
   * @brief 前回からの移動を基準方向に換算して積算する
   * @details reset後に呼ぶ。不正な入力では計測を無効にし、積算値を保持する。
   * @param mileage 車輪から求めた累計走行距離[mm]
   * @param angle 基準を0度とした現在方位[deg]
   */
  void update(double mileage, double angle);

  /**
   * @brief 基準方向への累積移動距離を取得する
   * @return 基準方向への符号付き移動距離[mm]
   */
  double getDistance() const;

  /** 基準方向をX、その+90度方向をYとした符号付き累積距離[mm] */
  double getHorizontalDistance() const { return distance; }
  double getVerticalDistance() const { return verticalDistance; }

  /**
   * @brief 起点が設定され、距離計測に異常がないかを取得する
   * @return true/計測可能、false/未初期化または異常（resetで復帰する）
   */
  bool isValid() const;

 private:
  bool valid = false;  // reset後は計測可能、入力異常時は次のresetまで無効
  double previousMileage = 0.0;   // 前回の累計走行距離[mm]
  double previousAngle = 0.0;     // 前回の方位[deg]
  double verticalDistance = 0.0;  // 基準方向と直交する方向への累積移動距離[mm]
  double distance = 0.0;          // 基準方向への累積移動距離[mm]
};

#endif
