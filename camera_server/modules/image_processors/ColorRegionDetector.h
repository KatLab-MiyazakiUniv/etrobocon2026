/**
 * @file ColorRegionDetector.h
 * @brief 色領域検出用の画像処理クラス
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef COLOR_REGION_DETECTOR_H
#define COLOR_REGION_DETECTOR_H

#include "BoundingBoxDetector.h"
#include "SystemInfo.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "Logger.h"

// HSVの範囲を保持する構造体
struct HSVRange {
  cv::Scalar lower;
  cv::Scalar upper;
};

class ColorRegionDetector : public BoundingBoxDetector {
 public:
  /**
   * @brief コンストラクタ
   * @param hsvRanges 検出したい色のHSV範囲
   * @param roi 注目領域
   */
  ColorRegionDetector(const std::vector<HSVRange>& hsvRanges, const cv::Rect& roi);
  /**
   * @brief デストラクタ
   */
  ~ColorRegionDetector();
  // フレームから色を検出する
  /**
   * @brief フレームから色領域を検出
   * @param frame 入力フレーム
   * @param result 検出結果
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result) override;

 private:
  // ノイズとみなす輪郭面積の閾値[px^2]
  static constexpr double MIN_CONTOUR_AREA = 50.0;
  // 検出したい色の範囲
  std::vector<HSVRange> hsvRanges;
  // 注目領域
  cv::Rect roi;
  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();
};

#endif
