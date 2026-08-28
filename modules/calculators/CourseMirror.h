/**
 * @file   CourseMirror.h
 * @brief  Lコースの座標・方向をRコースへ左右反転するクラス
 * @author yutaro-1214
 */

#ifndef COURSE_MIRROR_H
#define COURSE_MIRROR_H

#include "RouteTypes.h"
#include "SystemInfo.h"

class CourseMirror {
 public:
  /**
   * @brief X座標を左右反転する
   *
   * L:
   * 10 9 8 7 6 5 4 3 2 1 0
   *
   * R:
   *  0 1 2 3 4 5 6 7 8 9 10
   *
   * @param x Lコース側のX座標
   * @return Rコース側のX座標
   */
  static int mirrorX(int x);

  /**
   * @brief 座標を左右反転する
   *
   * Xのみ反転し、Yはそのまま使用する
   *
   * @param point 反転前の座標
   * @return 反転後の座標
   */
  static Point mirrorPoint(const Point& point);

  /**
   * @brief 方向を左右反転する
   *
   * LEFT  <-> RIGHT
   * UP    -> UP
   * DOWN  -> DOWN
   *
   * @param direction 反転前の方向
   * @return 反転後の方向
   */
  static Direction mirrorDirection(Direction direction);

 private:
  /**
   * @brief インスタンス化を禁止する
   */
  CourseMirror();
};

#endif  // COURSE_MIRROR_H