/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
  // QRコードのみを検出対象として設定
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  // QRコードを複数の向きやサイズで詳細に探索し、検出・デコードを行う
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

  // 処理対象のフレームからZXing用のImageViewを生成
  ZXing::ImageView iv(frame.data, frame.cols, frame.rows, ZXing::ImageFormat::BGR,
                      static_cast<int>(frame.step));

  // 処理対象のフレームからQRコードを検出し、デコード結果を取得
  auto qrCode = ZXing::ReadBarcode(iv, options);

  // QRコードが検出できなかった場合は終了
  if(!qrCode.isValid()) {
    Logger::error("QrCodeDetector: QRコードの検出に失敗しました。");
    return result;
  }

  // 検出結果を保存
  result.wasDetected = true;
  result.content = qrCode.text();

  // QRコードの4頂点座標を保存
  auto position = qrCode.position();
  result.corners[0] = cv::Point2f(position.topLeft().x, position.topLeft().y);
  result.corners[1] = cv::Point2f(position.topRight().x, position.topRight().y);
  result.corners[2] = cv::Point2f(position.bottomRight().x, position.bottomRight().y);
  result.corners[3] = cv::Point2f(position.bottomLeft().x, position.bottomLeft().y);

  return result;
}