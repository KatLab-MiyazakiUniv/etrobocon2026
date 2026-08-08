/**
 * @file Position.cpp
 * @brief ロボットの位置情報を保持するクラス
 * @author yutaro-1214
 */

#include "Position.h"

Position::Position() : x(0.0), y(0.0), heading(0.0) {}

void Position::set(double x, double y, double heading)
{
  this->x = x;
  this->y = y;
  this->heading = heading;
}

void Position::setX(double x)
{
  this->x = x;
}

void Position::setY(double y)
{
  this->y = y;
}

void Position::setHeading(double heading)
{
  this->heading = heading;
}

double Position::getX() const
{
  return x;
}

double Position::getY() const
{
  return y;
}

double Position::getHeading() const
{
  return heading;
}