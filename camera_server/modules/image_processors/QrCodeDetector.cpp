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

  if(!detector.detect(frame, corners)) {
    Logger::error("OpenCV detect failed");
    return result;
  }

  Logger::printfLog(Logger::INFO, "corners.size() = %zu", corners.size());

  if(corners.size() != 4) {
    Logger::error("corners.size() != 4");
    return result;
  }
  // 透視変換で正面化
  cv::Mat rectifiedFrame = rectify(frame, corners);

  // グレースケール化
  cv::Mat gray;
  cv::cvtColor(rectifiedFrame, gray, cv::COLOR_BGR2GRAY);

  // ノイズ除去
  cv::Mat denoised;
  cv::medianBlur(gray, denoised, 3);

  // シャープ化
  cv::Mat sharpened;
  cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);

  cv::filter2D(denoised, sharpened, -1, kernel);

  // 二値化
  cv::Mat binary;
  cv::adaptiveThreshold(sharpened, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                        21, 10);

  // 正面化したフレームをデコード
  ZXing::ImageView iv(binary.data, binary.cols, binary.rows, ZXing::ImageFormat::Lum,
                      static_cast<int>(binary.step));
  auto qrCode = ZXing::ReadBarcode(iv, options);

  if(!qrCode.isValid()) {
    Logger::error("ZXing decode failed");
    return result;
  }

  result.wasDetected = true;
  result.content = qrCode.text();

  for(int i = 0; i < 4; ++i) {
    result.corners[i] = corners[i];
  }
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
