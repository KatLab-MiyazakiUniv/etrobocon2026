/**
 * @file   QrCodeDetector.h
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTOR_H
#define QR_CODE_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <ZXing/ReadBarcode.h>
#include <ZXing/ReaderOptions.h>
#include "CodeDetector.h"
#include "QrCodeDetectionResult.h"
#include "Logger.h"

class QrCodeDetector : public CodeDetector<QrCodeDetectionResult> {
 public:
  /**
   * @brief コンストラクタ
   */
  QrCodeDetector();

  /**
   * @brief デストラクタ
   */
  ~QrCodeDetector();

  /**
   * @brief フレーム内のQRコードを検出する
   * @param frame 処理対象のフレーム
   * @return 検出結果（QrCodeDetectionResult型）
   */
  QrCodeDetectionResult detect(const cv::Mat& frame) override;

 private:
  static constexpr float quietZoneRatio = 0.2f;  // 透視変換後のクワイエットゾーン比率
  cv::QRCodeDetector detector;                   // OpenCVのQRコード検出器
  ZXing::ReaderOptions options;                  // ZXingのデコードオプション

  /**
   * @brief 検出したQRコードの各頂点の座標から透視変換でQRコード領域を正面化する
   * @param frame 処理対象のフレーム
   * @param corners 検出したQRコードの各頂点の座標
   * @return 正面化したQRコード領域のフレーム
   */
  cv::Mat rectify(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const;
};

#endif  // QR_CODE_DETECTOR_H
