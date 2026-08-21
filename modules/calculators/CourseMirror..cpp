/**
 * @file   CourseMirror.cpp
 * @brief  Lコースの座標・方向をRコースへ左右反転するクラス
 * @author yutaro-1214
 */

#include "CourseMirror.h"

int CourseMirror::mirrorX(int x)
{
  /*
   * X_GRID_NUM = 10 の場合
   *
   * L  -> R
   *
   * 10 -> 0
   *  9 -> 1
   *  8 -> 2
   *  7 -> 3
   *  6 -> 4
   *  5 -> 5
   *  4 -> 6
   *  3 -> 7
   *  2 -> 8
   *  1 -> 9
   *  0 -> 10
   */
  return SystemInfo::X_GRID_NUM - x;
}

Point CourseMirror::mirrorPoint(const Point& point)
{
  return { mirrorX(point.x), point.y };
}

Direction CourseMirror::mirrorDirection(Direction direction)
{
  switch(direction) {
    case Direction::LEFT:
      return Direction::RIGHT;

    case Direction::RIGHT:
      return Direction::LEFT;

    case Direction::UP:
      return Direction::UP;

    case Direction::DOWN:
      return Direction::DOWN;
  }

  return direction;
}