/**
 * @file   ImageRecognitionResults.h
 * @brief  画像認識結果のデータ構造定義
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef IMAGE_RECOGNITION_RESULTS_H
#define IMAGE_RECOGNITION_RESULTS_H

#include <cstdint>
#include <opencv2/core/types.hpp>

struct BoundingBoxDetectionResult {
  bool wasDetected = false;  // 検出できたかどうか
  cv::Point topLeft;         // 検出した領域の左上の座標
  cv::Point topRight;        // 検出した領域の右上の座標
  cv::Point bottomLeft;      // 検出した領域の左下の座標
  cv::Point bottomRight;     // 検出した領域の右下の座標
};

#endif  // IMAGE_RECOGNITION_RESULTS_H
