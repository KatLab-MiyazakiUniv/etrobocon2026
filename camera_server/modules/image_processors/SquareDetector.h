/**
 * @file   SquareDetector.h
 * @brief  正方形検出用の画像処理クラス
 * @author okuyama0528
 */

#ifndef SQUARE_DETECTOR_H
#define SQUARE_DETECTOR_H

#include <opencv2/opencv.hpp>

#include "CameraCapture.h"
#include "Logger.h"
#include "SocketProtocol.h"

/**
 * @brief 正方形検出結果
 */
struct BoundingBoxDetectionResult {
  /**
   * @brief 正方形を検出できたか
   */
  bool wasDetected = false;

  /**
   * @brief 左上
   */
  cv::Point topLeft;

  /**
   * @brief 右上
   */
  cv::Point topRight;

  /**
   * @brief 右下
   */
  cv::Point bottomRight;

  /**
   * @brief 左下
   */
  cv::Point bottomLeft;
};

/**
 * @brief 正方形を画像から検出するクラス
 *
 * 各detect呼び出しは独立して処理する。
 * 前回の正方形検出位置は使用しない。
 */
class SquareDetector {
 public:
  /**
   * @brief コンストラクタ
   *
   * @param _roi 正方形検出対象領域
   */
  explicit SquareDetector(const cv::Rect& _roi);

  /**
   * @brief デストラクタ
   */
  ~SquareDetector();

  /**
   * @brief 正方形を検出する
   *
   * 前回の検出結果は使用せず、
   * 渡されたフレームだけから正方形を検出する。
   *
   * @param frame 入力画像
   * @param result 正方形検出結果
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result);

  /**
   * @brief ROIを設定する
   *
   * 指定されたROIが画像範囲外の場合は、
   * 画像内に収まるように補正する。
   *
   * @param _roi 新しいROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  /**
   * @brief 正方形検出対象ROI
   */
  cv::Rect roi;

  /**
   * @brief 最小輪郭面積
   *
   * この値より小さい輪郭は
   * ノイズとして除外する。
   */
  static constexpr double MIN_CONTOUR_AREA = 100.0;

  /**
   * @brief 正方形として許容する最小縦横比
   *
   * min(width, height) / max(width, height)
   *
   * 1.0に近いほど正方形。
   *
   * 例:
   *
   * 1.00
   *   完全な正方形
   *
   * 0.80
   *   多少変形した正方形も許可
   */
  static constexpr double MIN_RATIO = 0.75;

  /**
   * @brief 外接矩形に対する最低面積割合
   *
   * contourArea / rotatedRectArea
   *
   * 値が大きいほど、
   * 外接矩形をしっかり埋めている形だけを
   * 正方形として扱う。
   */
  static constexpr double MIN_FILL_RATIO = 0.70;

  /**
   * @brief 許容する最大円形度
   *
   * 円形度:
   *
   * 4 * PI * area / perimeter^2
   *
   * 円:
   *   約1.0
   *
   * 正方形:
   *   約0.785
   *
   * 円に近すぎる輪郭を除外する。
   */
  static constexpr double MAX_CIRCULARITY = 0.92;

  /**
   * @brief ROIパラメータを画像範囲内に補正する
   */
  void validateParameters();
};

#endif  // SQUARE_DETECTOR_H