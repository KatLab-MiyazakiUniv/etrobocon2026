/*
 * @file SquareDetector.h
 * @brief 正方形検出用の画像処理クラス
 * @author okuyama0528
 */

#ifndef SQUARE_DETECTOR_H
#define SQUARE_DETECTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <climits>
#include "Logger.h"
#include "BoundingBoxDetector.h"
#include "SystemInfo.h"

class SquareDetector : public BoundingBoxDetector {
 public:
  /**
   * @brief コンストラクタ
   * @param roi 注目領域
   */
  explicit SquareDetector(const cv::Rect& roi);

  /**
   * @brief デストラクタ
   */
  ~SquareDetector();

  /**
   * @brief フレームから正方形を検出
   * @param frame 入力フレーム
   * @param result 検出結果
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result) override;

  /**
   * @brief ROIを検証したうえで設定する
   * @param _roi 設定するROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  cv::Rect roi;  // フレーム全体に対するROI
  static constexpr double MIN_CONTOUR_AREA
      = 50.0;  // これ以下の面積の輪郭はノイズとみなして除外する

  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();
};
#endif  // SQUARE_DETECTOR_H