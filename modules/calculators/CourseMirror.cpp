/**
 * @file   CourseMirror.cpp
 * @brief  Lコースの座標・方向をRコースへ左右反転するクラス
 * @author yutaro-1214 okuyama0528
 */

#include "CourseMirror.h"

int CourseMirror::mirrorX(int x)
{
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
Logger::warning("CourseMirror:Invalid direction");
  return direction;
}