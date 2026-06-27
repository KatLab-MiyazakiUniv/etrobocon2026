/**
 * @file ImageProcessingColor.cpp
 * @brief 画像処理で使用する色定義とHSV変換処理
 * @author takuchi17
 */

#include "ImageProcessingColor.h"

const CameraServer::HSVRangeData ImageProcessingColor::RED_HSV
    = { { 0.0, 100.0, 100.0 }, { 10.0, 255.0, 255.0 } };
const CameraServer::HSVRangeData ImageProcessingColor::BLUE_HSV
    = { { 100.0, 100.0, 100.0 }, { 130.0, 255.0, 255.0 } };
const CameraServer::HSVRangeData ImageProcessingColor::GREEN_HSV
    = { { 40.0, 100.0, 100.0 }, { 80.0, 255.0, 255.0 } };
const CameraServer::HSVRangeData ImageProcessingColor::YELLOW_HSV
    = { { 20.0, 100.0, 100.0 }, { 35.0, 255.0, 255.0 } };
const CameraServer::HSVRangeData ImageProcessingColor::BLACK_HSV
    = { { 0.0, 0.0, 0.0 }, { 180.0, 255.0, 50.0 } };
const CameraServer::HSVRangeData ImageProcessingColor::GRAY_HSV
    = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
const CameraServer::HSVRangeData ImageProcessingColor::WHITE_HSV
    = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };

const CameraServer::HSVRangeData& ImageProcessingColor::getHSVRangeFromColor(Color color)
{
  switch(color) {
    case RED:
      return RED_HSV;
    case BLUE:
      return BLUE_HSV;
    case GREEN:
      return GREEN_HSV;
    case YELLOW:
      return YELLOW_HSV;
    case BLACK:
      return BLACK_HSV;
    case GRAY:
      return GRAY_HSV;
    case WHITE:
      return WHITE_HSV;
    default:
      return RED_HSV;
  }
}