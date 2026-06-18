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
   */
  void set(double x, double y, double heading);

  /**
   * @brief X座標設定
   */
  void setX(double x);

  /**
   * @brief Y座標設定
   */
  void setY(double y);

  /**
   * @brief 方位角設定
   */
  void setHeading(double heading);

  double getX() const;
  double getY() const;
  double getHeading() const;

 private:
  double x;
  double y;
  double heading;
};

#endif