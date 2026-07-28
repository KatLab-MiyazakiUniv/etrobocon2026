/**
 * @file   QrCodeDetector.h
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTOR_H
#define QR_CODE_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ZXing/ReadBarcode.h>
#include <ZXing/ReaderOptions.h>
#include "CodeDetector.h"
#include "Logger.h"
#include "QrCodeDetectionResult.h"

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
   * @brief フレーム内のQRコードを検出・デコードする
   * @param frame 処理対象のフレーム
   * @return 検出結果
   */
  QrCodeDetectionResult detect(const cv::Mat& frame) override;

 private:
  ZXing::ReaderOptions options;  // ZXingのデコードオプション
};

#endif  // QR_CODE_DETECTOR_H