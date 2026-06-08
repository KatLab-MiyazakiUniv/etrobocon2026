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
#include <cstdint>
#include <opencv2/opencv.hpp>
#include "Logger.h"

class ColorRegionDetector : public BoundingBoxDetector {
 public:
  /**
   * @brief コンストラクタ
   * @param hsvRanges 検出したい色のHSV範囲
   * @param roi 注目領域
   */
  ColorRegionDetector(const std::vector<ColorRegionDetector::HSVRange>& hsvRanges,
                      const cv::Rect& roi);

  /**
   * @brief デストラクタ
   */
  ~ColorRegionDetector();

  // HSVの範囲を保持する構造体
  struct HSVRange {
    cv::Scalar lower;  // HSVの下限値
    cv::Scalar upper;  // HSVの上限値
  };

  /**
   * @brief フレームから色領域を検出
   * @param frame 入力フレーム
   * @param result 検出結果
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result) override;

  /**
   * @brief フレームから色領域を検出し、最も面積が大きい色のインデックスも取得
   * @param frame 入力フレーム
   * @param result 検出結果
   * @param largestColorIndex 最も面積が大きい色のインデックス
   * @details 検出できなかった場合は largestColorIndex に -1 を設定する
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result, int32_t& largestColorIndex);

 private:
  static constexpr double MIN_CONTOUR_AREA
      = 50.0;                       // これ以下の面積の輪郭はノイズとみなして除外する
  std::vector<HSVRange> hsvRanges;  // 検出したい色のHSV範囲のリスト
  cv::Rect roi;                     // フレーム全体に対するROI
  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();

  /**
   * @brief 色領域のバウンディングボックスを検出する
   * @param frame 入力フレーム
   * @param result 検出結果
   * @param hsvFrame ROI内のHSV画像
   * @param roiRect フレーム内に収めたROI
   * @param boundingBox ROI内座標系のバウンディングボックス
   * @return 検出できた場合 true、できなかった場合 false
   * @details 複数色のマスクを統合して、統合された色領域全体のバウンディングボックスを検出する
   */
  bool detectBoundingBox(const cv::Mat& frame, BoundingBoxDetectionResult& result,
                         cv::Mat& hsvFrame, cv::Rect& roiRect, cv::Rect& boundingBox);

  /**
   * @brief 検出されたバウンディングボックス内で最も面積が大きい色のインデックスを取得する
   * @param hsvFrame ROI内のHSV画像
   * @param boundingBox ROI内座標系のバウンディングボックス
   * @return 最も面積が大きい色のインデックス。見つからない場合は -1
   */
  int32_t findLargestColorIndex(const cv::Mat& hsvFrame, const cv::Rect& boundingBox);

  /**
   * @brief バウンディングボックス情報を検出結果に設定する
   * @param boundingBox ROI内座標系のバウンディングボックス
   * @param roiRect フレーム全体に対するROI
   * @param result 検出結果
   */
  void setBoundingBoxResult(const cv::Rect& boundingBox, const cv::Rect& roiRect,
                            BoundingBoxDetectionResult& result);
};

#endif