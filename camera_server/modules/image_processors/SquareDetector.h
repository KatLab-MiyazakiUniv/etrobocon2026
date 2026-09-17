/**
 * @file   SquareDetector.h
 * @brief  正方形検出用の画像処理クラス
 * @author okuyama0528 yutaro-1214
 */

#ifndef SQUARE_DETECTOR_H
#define SQUARE_DETECTOR_H

#include <opencv2/opencv.hpp>

#include "CameraCapture.h"
#include "ImageRecognitionResults.h"
#include "Logger.h"
#include "SocketProtocol.h"

/**
 * @brief 正方形状の領域を画像から検出するクラス
 */
class SquareDetector {
 public:
  /**
   * @brief コンストラクタ
   * @param _roi 検出対象領域
   */
  explicit SquareDetector(const cv::Rect& _roi);

  /**
   * @brief デストラクタ
   */
  ~SquareDetector();

  /**
   * @brief 正方形状の領域を検出する
   *
   * @param frame 入力画像
   * @param result 検出結果
   */
  void detect(
      const cv::Mat& frame,
      BoundingBoxDetectionResult& result);

  /**
   * @brief ROIを設定する
   *
   * @param _roi 新しいROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  /**
   * @brief 検出対象ROI
   */
  cv::Rect roi;

  /**
   * @brief 最小輪郭面積
   *
   * 小さいノイズやQR内部の細かい模様を除外する。
   */
  static constexpr double MIN_CONTOUR_AREA = 2000.0;

  /**
   * @brief 最小縦横比
   *
   * カメラの遠近によって横長に見える場合を考慮する。
   */
  static constexpr double MIN_RATIO = 0.40;

  /**
   * @brief 最小充填率
   */
  static constexpr double MIN_FILL_RATIO = 0.60;

  /**
   * @brief 最大円形度
   *
   * 円に近い輪郭を除外する。
   */
  static constexpr double MAX_CIRCULARITY = 0.82;

  /**
   * @brief ROIを画像範囲内に補正する
   */
  void validateParameters();
};

#endif  // SQUARE_DETECTOR_H