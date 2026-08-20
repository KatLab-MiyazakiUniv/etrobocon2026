/**
 * @file   QrCodeDetector.h
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTOR_H
#define QR_CODE_DETECTOR_H

#include <opencv2/core.hpp>
#include <ZXing/ReadBarcode.h>
#include <ZXing/ReaderOptions.h>
#include "CodeDetector.h"
#include "Logger.h"
#include "QrCodeDetectionResult.h"
#include "SystemInfo.h"

class QrCodeDetector : public CodeDetector<QrCodeDetectionResult> {
 public:
  /**
   * @brief コンストラクタ
   * @param roi 注目領域
   */
  explicit QrCodeDetector(const cv::Rect& roi);

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

  /**
   * @brief ROIを検証したうえで設定する
   * @param _roi 設定するROI
   */
  void setValidatedRoi(const cv::Rect& _roi);

 private:
  ZXing::ReaderOptions options;  // ZXingのデコードオプション
  cv::Rect roi;                  // フレーム全体に対するROI

  /**
   * @brief ROIがフレーム内に収まるように補正する
   */
  void validateParameters();
};

#endif  // QR_CODE_DETECTOR_H