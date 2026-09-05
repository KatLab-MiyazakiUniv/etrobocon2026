/**
 * @file   GatePosition.cpp
 * @brief  ゲート位置情報を管理するクラス
 * @author migaku2645
 */

#include "GatePosition.h"

GatePosition::GatePosition() : red{ 0, 0 }, blue{ 0, 0 }, yellow{ 0, 0 }
{
  LOG_CREATE("GatePosition");
}

void GatePosition::setPosition(GateColor color, int position1, int position2)
{
  switch(color) {
    case GateColor::RED:
      red[0] = position1;
      red[1] = position2;
      break;

    case GateColor::BLUE:
      blue[0] = position1;
      blue[1] = position2;
      break;

    case GateColor::YELLOW:
      yellow[0] = position1;
      yellow[1] = position2;
      break;

    default:
      Logger::error("GatePosition: 不正なゲート色が指定されました。");
      break;
  }
}

void GatePosition::getPosition(GateColor color, int position[2]) const
{
  switch(color) {
    case GateColor::RED:
      position[0] = red[0];
      position[1] = red[1];
      break;

    case GateColor::BLUE:
      position[0] = blue[0];
      position[1] = blue[1];
      break;

    case GateColor::YELLOW:
      position[0] = yellow[0];
      position[1] = yellow[1];
      break;

    default:
      Logger::error("GatePosition: 不正なゲート色が指定されました。");

      position[0] = -1;
      position[1] = -1;
      break;
  }
}