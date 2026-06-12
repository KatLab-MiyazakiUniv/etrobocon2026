/**
 * @file   QrCodeDetector.cpp
 * @brief  QRコード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
  detector.setEpsX(0.2);  // 水平方向のファインダパターン探索の許容誤差指定
  detector.setEpsY(0.2);  // 垂直方向のファインダパターン探索の許容誤差指定
  options.setFormats(ZXing::BarcodeFormat::QRCode);  // QRコードのみを検出
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

  // QRコードの各頂点の位置を検出
  std::vector<cv::Point2f> corners;
  if(!detector.detect(frame, corners) || corners.size() != 4) {
    return result;
  }

  // 透視変換で正面化
  cv::Mat rectifiedFrame = rectify(frame, corners);

  // 正面化したフレームをデコード
  ZXing::ImageView iv(rectifiedFrame.data, rectifiedFrame.cols, rectifiedFrame.rows,
                      ZXing::ImageFormat::BGR, static_cast<int>(rectifiedFrame.step));
  auto qrCode = ZXing::ReadBarcode(iv, options);

  if(qrCode.isValid()) {
    result.wasDetected = true;
    result.content = qrCode.text();
    for(int i = 0; i < 4; ++i) {
      result.corners[i] = corners[i];
    }
  }
  return result;
}
cv::Mat QrCodeDetector::rectify(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const
{
  // 対辺の長さの最大値を正方形の一辺とする
  float maxWidth = std::max(cv::norm(corners[1] - corners[0]), cv::norm(corners[2] - corners[3]));
  float maxHeight = std::max(cv::norm(corners[3] - corners[0]), cv::norm(corners[2] - corners[1]));
  float outputQrSize = std::max(maxWidth, maxHeight);

  // クワイエットゾーン（余白）を追加する
  float quietZoneSize = outputQrSize * quietZoneRatio;
  int outputSize = static_cast<int>(outputQrSize + 2.f * quietZoneSize);

  // 余白分だけ内側にQRコード領域を配置
  std::vector<cv::Point2f> dstCorners
      = { { quietZoneSize, quietZoneSize },
          { quietZoneSize + outputQrSize, quietZoneSize },
          { quietZoneSize + outputQrSize, quietZoneSize + outputQrSize },
          { quietZoneSize, quietZoneSize + outputQrSize } };
  cv::Mat pixelNum = cv::getPerspectiveTransform(corners, dstCorners);
  cv::Mat rectifiedFrame;
  cv::warpPerspective(frame, rectifiedFrame, pixelNum, cv::Size(outputSize, outputSize));
  return rectifiedFrame;
}
