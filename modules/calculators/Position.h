/**
 * @file Position.h
 * @brief ロボットの位置情報を保持するクラス
 * @author yutaro-1214
 */

#ifndef POSITION_H
#define POSITION_H

class Position {
 public:
  Position();

  /**
   * @brief 位置と方位を設定
   * @param x 設定するX座標の値
   * @param y 設定するY座標の値
   * @param heading 設定する向きの値
   */
  void set(double x, double y, double heading);

  /**
   * @brief X座標設定
   * @param x 設定するX座標の値
   */
  void setX(double x);

  /**
   * @brief Y座標設定
   * @param y 設定するY座標の値
   */
  void setY(double y);

  /**
   * @brief 方位角設定
   * @param heading 設定する向きの値
   */
  void setHeading(double heading);

  /**
   * @brief X座標取得
   * @return 現在のX座標
   */
  double getX() const;

  /**
   * @brief Y座標取得
   * @return 現在のY座標
   */
  double getY() const;

  /**
   * @brief 機体の向き取得
   * @return 現在の機体の向き
   */
  double getHeading() const;

 private:
  double x;        // 機体のX座標
  double y;        // 機体のY座標
  double heading;  // 機体の向き
};

#endif