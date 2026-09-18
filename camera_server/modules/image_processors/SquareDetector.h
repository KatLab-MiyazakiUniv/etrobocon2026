/**
 * @file   SquareDetector.h
 * @brief  正方形検出用の画像処理クラス
 * @author okuyama0528 yutaro-1214
 */

#ifndef SQUARE_DETECTOR_H
#define SQUARE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>
#include "CameraCapture.h"
#include "ImageRecognitionResults.h"
#include "Logger.h"
#include "SocketProtocol.h"

/**
 * @brief 正方形を画像から検出するクラス
 */
class SquareDetector {
 public:
  /**
   * @brief コンストラクタ
   * @param _roi 正方形検出対象領域
   */
  explicit SquareDetector(const cv::Rect& _roi);

  /**
   * @brief デストラクタ
   */
  ~SquareDetector();

  /**
   * @brief 正方形を検出する
   * @param frame 入力画像
   * @param result 正方形検出結果
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result);

  /**
   * @brief ROIを設定する
   * @param _roi 新しいROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  cv::Rect roi;  // 正方形検出対象領域

  static constexpr double MIN_CONTOUR_AREA = 100.0;  // 最小輪郭面積

  /**
   * @brief 正方形として許容する最小縦横比
   * 1.0に近いほど正方形
   */
  static constexpr double MIN_RATIO = 0.75;

  /**
   * @brief 外接矩形に対する最低面積割合
   */
  static constexpr double MIN_FILL_RATIO = 0.60;

  /**
   * @brief 許容する最大円形度
   */
  static constexpr double MAX_CIRCULARITY = 0.82;

  /**
   * @brief ROIパラメータを画像範囲内に補正する
   */
  void validateParameters();
};

#endif  // SQUARE_DETECTOR_H