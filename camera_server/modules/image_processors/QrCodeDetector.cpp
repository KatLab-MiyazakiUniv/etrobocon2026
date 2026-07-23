/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  options.setTryHarder(true);  // 任意（検出率向上）
  LOG_CREATE("QrCodeDetector");
}

QrCodeDetector::~QrCodeDetector()
{
  LOG_DESTROY("QrCodeDetector");
}

QrCodeDetectionResult QrCodeDetector::detect(const cv::Mat& frame)
{
  QrCodeDetectionResult result;

  if(frame.empty()) {
    Logger::error("QrCodeDetector: 入力フレームが空です。");
    return result;
  }

  // ZXingだけで検出・デコード
  ZXing::ImageView iv(frame.data, frame.cols, frame.rows, ZXing::ImageFormat::BGR,
                      static_cast<int>(frame.step));

  auto qrCode = ZXing::ReadBarcode(iv, options);

  if(!qrCode.isValid()) {
    Logger::error("ZXing detect + decode failed");
    return result;
  }

  result.wasDetected = true;
  result.content = qrCode.text();

  return result;
}