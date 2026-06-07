/**
 * @file ImageProcessingColor.cpp
 * @brief 画像処理で使用する色定義とHSV変換処理
 * @author takuchi17
 */

#include "ImageProcessingColor.h"

const ImageProcessingColor::HSVRange ImageProcessingColor::RED_HSV
    = { cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255) };

const ImageProcessingColor::HSVRange ImageProcessingColor::BLUE_HSV
    = { cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255) };

const ImageProcessingColor::HSVRange ImageProcessingColor::GREEN_HSV
    = { cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255) };

const ImageProcessingColor::HSVRange ImageProcessingColor::YELLOW_HSV
    = { cv::Scalar(20, 100, 100), cv::Scalar(35, 255, 255) };

const ImageProcessingColor::HSVRange ImageProcessingColor::BLACK_HSV
    = { cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50) };

const ImageProcessingColor::HSVRange& ImageProcessingColor::getHSVRangeFromColor(Color color)
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

    default:
      return RED_HSV;
  }
}