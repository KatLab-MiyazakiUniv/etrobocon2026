/**
 * @file   QrCodeDetector.h
 * @brief  2次元コード検出処理クラス
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTOR_H
#define QR_CODE_DETECTOR_H

#include <opencv2/objdetect.hpp>
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
   * @brief フレーム内のすべての2次元コードを検出する
   * @param frame 処理対象のフレーム
   * @return 検出結果（QrCodeDetectionResult型）
   */
  QrCodeDetectionResult detect(const cv::Mat& frame) override;

 private:
  cv::QRCodeDetector detector;  // 2次元コード検出クラスのインスタンス
};

#endif  // QR_CODE_DETECTOR_H
