/**
 * @file ImageProcessingColor.h
 * @brief 画像処理で使用する色定義とHSV変換処理
 * @author takuchi17
 */

#ifndef IMAGE_PROCESSING_COLOR_H
#define IMAGE_PROCESSING_COLOR_H

#include <opencv2/core/types.hpp>

class ImageProcessingColor {
 public:
  /**
   * @brief 画像処理で使用する色
   */
  enum Color {
    RED,     // 赤
    BLUE,    // 青
    GREEN,   // 緑
    YELLOW,  // 黄
    BLACK    // 黒
  };

  /**
   * @brief HSV範囲を表す構造体
   */
  struct HSVRange {
    cv::Scalar lower;  // HSV下限値
    cv::Scalar upper;  // HSV上限値
  };

  /**
   * @brief 色に対応するHSV範囲を取得する
   * @param color 画像処理用の色
   * @return HSV範囲
   */
  static const HSVRange& getHSVRangeFromColor(Color color);

 private:
  /**
   * @brief インスタンス化禁止
   */
  ImageProcessingColor();

  /**
   * @brief 赤色のHSV範囲
   */
  static const HSVRange RED_HSV;

  /**
   * @brief 青色のHSV範囲
   */
  static const HSVRange BLUE_HSV;

  /**
   * @brief 緑色のHSV範囲
   */
  static const HSVRange GREEN_HSV;

  /**
   * @brief 黄色のHSV範囲
   */
  static const HSVRange YELLOW_HSV;

  /**
   * @brief 黒色のHSV範囲
   */
  static const HSVRange BLACK_HSV;
};

#endif  // IMAGE_PROCESSING_COLOR_H