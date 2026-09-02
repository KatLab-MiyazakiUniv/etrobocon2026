/**
 * @file   CourseMirror.h
 * @brief  Lコースの座標・方向をRコースへ左右反転するクラス
 * @author yutaro-1214 okuyama0528
 */

#ifndef COURSE_MIRROR_H
#define COURSE_MIRROR_H

#include "RouteTypes.h"
#include "SystemInfo.h"
#include "Logger.h"

class CourseMirror {
 public:
  /**
   * @brief X座標を左右反転する
   * @param x Lコース側のX座標
   * @return Rコース側のX座標
   */
  static int mirrorX(int x);

  /**
   * @brief 座標を左右反転する
   * @param point 反転前の座標
   * @return 反転後の座標
   */
  static Point mirrorPoint(const Point& point);

  /**
   * @brief ロボットの方向を左右反転する
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