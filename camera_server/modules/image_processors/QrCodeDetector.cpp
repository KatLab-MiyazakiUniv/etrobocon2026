/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  options.setTryHarder(true);
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

  // 入力画像からZXing用のImageViewを生成
  ZXing::ImageView iv(frame.data, frame.cols, frame.rows, ZXing::ImageFormat::BGR,
                      static_cast<int>(frame.step));

  // 入力画像からQRコードを検出し、デコード結果を取得
  auto qrCode = ZXing::ReadBarcode(iv, options);

  // QRコードが検出できなかった場合は終了
  if(!qrCode.isValid()) {
    Logger::error("ZXing detect + decode failed");
    return result;
  }

  // 検出結果を保存
  result.wasDetected = true;
  result.content = qrCode.text();

  return result;
}