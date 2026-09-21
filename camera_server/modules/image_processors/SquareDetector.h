/**
 * @file   SquareDetector.h
 * @brief  ETラリー上のQRコードを点の集合から正方形領域として検出する画像処理クラス
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
 * @brief QRコードを点の集合から正方形領域として捉えるクラス
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
   * @brief 正方形を検出する
   * @param frame 入力画像
   * @param result 検出結果
   */
  void detect(const cv::Mat& frame, BoundingBoxDetectionResult& result);

  /**
   * @brief ROIを検証したうえで設定する
   * @param _roi 設定するROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  cv::Rect roi;  // フレーム全体に対するROI

  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();
};

#endif  // SQUARE_DETECTOR_H