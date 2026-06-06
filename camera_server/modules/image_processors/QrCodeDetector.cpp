/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
  detector.setEpsX(0.2);
  detector.setEpsY(0.2);
  options.setFormats(ZXing::BarcodeFormat::QRCode);
  LOG_CREATE("QrCodeDetector");
}

QrCodeDetector::~QrCodeDetector()
{
  LOG_DESTROY("QrCodeDetector");
}

cv::Mat QrCodeDetector::rectify(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const
{
  // 対辺の長さの最大値を正方形の一辺とする
  float w = std::max(cv::norm(corners[1] - corners[0]), cv::norm(corners[2] - corners[3]));
  float h = std::max(cv::norm(corners[3] - corners[0]), cv::norm(corners[2] - corners[1]));
  float s = std::max(w, h);

  // クワイエットゾーン（余白）を20%分追加する
  float p = s / 5.f;
  int totalSize = static_cast<int>(s + 2.f * p);

  // 余白分だけ内側にQRコード領域を配置
  std::vector<cv::Point2f> dst = { { p, p }, { p + s, p }, { p + s, p + s }, { p, p + s } };
  cv::Mat M = cv::getPerspectiveTransform(corners, dst);
  cv::Mat rectified;
  cv::warpPerspective(frame, rectified, M, cv::Size(totalSize, totalSize));
  return rectified;
}

QrCodeDetectionResult QrCodeDetector::detect(const cv::Mat& frame)
{
  QrCodeDetectionResult result;

  if(frame.empty()) {
    Logger::error("QrCodeDetector: 入力フレームが空です。");
    return result;
  }

  // QRコードの各頂点の位置を検出
  std::vector<cv::Point2f> corners;
  if(!detector.detect(frame, corners) || corners.size() < 4) {
    return result;
  }

  // 透視変換で正面化
  cv::Mat rectified = rectify(frame, corners);

  // 正面化したフレームをデコード
  ZXing::ImageView iv(rectified.data, rectified.cols, rectified.rows, ZXing::ImageFormat::BGR,
                      static_cast<int>(rectified.step));
  auto qrCode = ZXing::ReadBarcode(iv, options);

  if(qrCode.isValid()) {
    result.wasDetected = true;
    result.gatePosition = qrCode.text();
    for(int i = 0; i < 4; ++i) {
      result.corners[i] = cv::Point(static_cast<int>(corners[i].x), static_cast<int>(corners[i].y));
    }
  }

  return result;
}
