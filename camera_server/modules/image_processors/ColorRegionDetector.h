/**
 * @file ColorRegionDetector.h
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528 sadomiya-sousi
 */

#ifndef COLOR_REGION_DETECTOR_H
#define COLOR_REGION_DETECTOR_H

#include "BoundingBoxDetector.h"
#include "SystemInfo.h"
#include <vector>
#include <opencv2/opencv.hpp>

// 色領域検出クラスの設定用構造体
struct ColorRegionDetectorConfig {
  std::vector<cv::Scalar> lowerHSV;
  std::vector<cv::Scalar> upperHSV;
  cv::Rect roi;
  cv::Size resolution;
};

// 色領域検出クラス
class ColorRegionDetector : public BoundingBoxDetector {
 public:
  // コンストラクタ
  // 初期設定
  explicit ColorRegionDetector(const ColorRegionDetectorConfig& config);
  // 画像から色を検出する
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result) override;

 private:
  // 小さすぎるノイズを無視する基準
  static constexpr double MIN_LINE_CONTOUR_AREA = 50.0;
  // 検出したい色の範囲
  std::vector<cv::Scalar> lowerHSV;
  std::vector<cv::Scalar> upperHSV;
  // 注目領域
  cv::Rect roi;
  // 解像度
  cv::Size resolution;

  void validateParameters();
};

#endif