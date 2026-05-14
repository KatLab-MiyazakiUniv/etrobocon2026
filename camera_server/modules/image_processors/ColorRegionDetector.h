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

// HSVの範囲を保持する構造体
struct HSVRange {
  cv::Scalar lower;
  cv::Scalar upper;
};

// 色領域検出クラス
class ColorRegionDetector : public BoundingBoxDetector {
 public:
  // デフォルト引数では無く,コンストラクタのオーバーロードで複数の初期化方法を提供する設計
  // 1. 全てのパラメータを個別に指定する
  ColorRegionDetector(const std::vector<HSVRange>& hsvRanges, const cv::Rect& roi,
                      const cv::Size& resolution);

  // 2. 複数色の範囲とROIを指定する (解像度はデフォルト 640x480)
  ColorRegionDetector(const std::vector<HSVRange>& hsvRanges, const cv::Rect& roi);

  // 3. 複数色の範囲のみ指定する (デフォルトのROI 50,240,540,240 および解像度 640x480 を使用)
  explicit ColorRegionDetector(const std::vector<HSVRange>& hsvRanges);

  // フレームから色を検出する
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result) override;

 private:
  // ノイズとみなす輪郭面積の閾値[px^2]
  static constexpr double MIN_CONTOUR_AREA = 50.0;
  // 検出したい色の範囲
  std::vector<HSVRange> hsvRanges;
  // 注目領域
  cv::Rect roi;
  // 解像度
  cv::Size resolution;

  void validateParameters();
};

#endif