/**
 * @file ImageProcessingColor.h
 * @brief 画像処理で使用する色定義とHSV変換処理
 * @author takuchi17
 */

#ifndef IMAGE_PROCESSING_COLOR_H
#define IMAGE_PROCESSING_COLOR_H

#include "SocketProtocol.h"

/**
 * @brief 画像処理で使用する色定義とHSV変換処理を提供するクラス
 */
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
   * @brief 色に対応するHSV範囲を取得する
   * @param color 画像処理用の色
   * @return HSV範囲
   */
  static const CameraServer::HSVRangeData& getHSVRangeFromColor(Color color);

 private:
  /**
   * @brief インスタンス化禁止
   */
  ImageProcessingColor();

  static const CameraServer::HSVRangeData RED_HSV;     // 赤色のHSV範囲
  static const CameraServer::HSVRangeData BLUE_HSV;    // 青色のHSV範囲
  static const CameraServer::HSVRangeData GREEN_HSV;   // 緑色のHSV範囲
  static const CameraServer::HSVRangeData YELLOW_HSV;  // 黄色のHSV範囲
  static const CameraServer::HSVRangeData BLACK_HSV;   // 黒色のHSV範囲
};

#endif  // IMAGE_PROCESSING_COLOR_H