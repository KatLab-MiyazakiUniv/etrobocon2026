/**
 * @file   SquareDetector.h
 * @brief  正方形検出用の画像処理クラス
 * @author okuyama0528
 */

#ifndef SQUARE_DETECTOR_H
#define SQUARE_DETECTOR_H

#include <vector>

#include <opencv2/opencv.hpp>

#include "BoundingBoxDetector.h"
#include "Logger.h"
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
  void detect(
      const cv::Mat& frame,
      BoundingBoxDetectionResult& result) override;

  /**
   * @brief ROIを検証したうえで設定する
   * @param _roi 設定するROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  /**
   * @brief フレーム全体に対するROI
   */
  cv::Rect roi;

  /**
   * @brief 検出対象とする最小輪郭面積
   */
  static constexpr double MIN_CONTOUR_AREA = 100.0;

  /**
   * @brief 正方形らしさの最小値
   *
   * min(width, height) / max(width, height)
   */
  static constexpr double MIN_RATIO = 0.6;

  /**
   * @brief 外接矩形に対する輪郭面積の最小割合
   */
  static constexpr double MIN_FILL_RATIO = 0.82;

  /**
   * @brief 円形度の最大値
   *
   * 円は1.0に近く、
   * 正方形は約0.785。
   */
  static constexpr double MAX_CIRCULARITY = 0.88;

  /**
   * @brief 前フレームから許容する最大中心移動量[pixel]
   *
   * これを超えて移動した候補は誤検出とみなす。
   */
  static constexpr double MAX_CENTER_MOVE = 80.0;

  /**
   * @brief 正方形を見失った場合に保持する最大フレーム数
   */
  static constexpr int MAX_MISSED_FRAMES = 5;

  /**
   * @brief 前回検出した正方形の中心位置
   */
  cv::Point2f previousCenter;

  /**
   * @brief 前回の正方形検出結果を保持しているか
   */
  bool hasPreviousDetection;

  /**
   * @brief 正方形を連続して見失ったフレーム数
   */
  int missedFrames;

  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();
};

#endif  // SQUARE_DETECTOR_H