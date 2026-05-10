
/**
 * @file ColorRegionDetector.h
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528 sadomiya
 */

#ifndef COLOR_REGION_DETECTOR_H
#define COLOR_REGION_DETECTOR_H

#include "BoundingBoxDetector.h"
#include "SystemInfo.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "CameraCapture.h"

// 色領域検出クラス（複数色対応）
class ColorRegionDetector {
 public:
  // コンストラクタ（複数色対応）
  ColorRegionDetector(const std::vector<cv::Scalar>& _lowerHSV,
                      const std::vector<cv::Scalar>& _upperHSV, const cv::Rect& _roi,
                      const cv::Size& _resolution);

  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result);

 private:
  static constexpr double MIN_LINE_CONTOUR_AREA = 50.0;

  std::vector<cv::Scalar> lowerHSV;  // 複数色対応
  std::vector<cv::Scalar> upperHSV;  // 複数色対応

  cv::Rect roi;         // 注目領域
  cv::Size resolution;  // 解像度

  void validateParameters();
};

#endif