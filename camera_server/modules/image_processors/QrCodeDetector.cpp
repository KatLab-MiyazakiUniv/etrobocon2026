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

cv::Mat QrCodeDetector::rectify(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const
{
  // 対辺の長さの最大値を正方形の一辺とする
  float maxWidth = std::max(cv::norm(corners[1] - corners[0]), cv::norm(corners[2] - corners[3]));
  float maxHeight = std::max(cv::norm(corners[3] - corners[0]), cv::norm(corners[2] - corners[1]));
  float qrSide = std::max(maxWidth, maxHeight);

  // クワイエットゾーン（余白）を追加する
  float quietZoneSize = qrSide * quietZoneRatio;
  int outputSize = static_cast<int>(qrSide + 2.f * quietZoneSize);

  // 余白分だけ内側にQRコード領域を配置
  std::vector<cv::Point2f> dstCorners = { { quietZoneSize, quietZoneSize },
                                          { quietZoneSize + qrSide, quietZoneSize },
                                          { quietZoneSize + qrSide, quietZoneSize + qrSide },
                                          { quietZoneSize, quietZoneSize + qrSide } };
  cv::Mat pixelNum = cv::getPerspectiveTransform(corners, dstCorners);
  cv::Mat rectified;
  cv::warpPerspective(frame, rectified, pixelNum, cv::Size(outputSize, outputSize));
  return rectified;
}

std::vector<QrCodeDetectionResult> QrCodeDetector::detect(const cv::Mat& frame)
{
  std::vector<QrCodeDetectionResult> results;

  if(frame.empty()) {
    Logger::error("QrCodeDetector: 入力フレームが空です。");
    return results;
  }

  // フレーム内のすべてのQRコードの頂点を検出
  std::vector<cv::Point2f> allPoints;
  if(!detector.detectMulti(frame, allPoints)) {
    return results;
  }

  for(size_t i = 0; i + 4 <= allPoints.size(); i += 4) {
    std::vector<cv::Point2f> corners(allPoints.begin() + i, allPoints.begin() + i + 4);

    // 透視変換で正面化
    cv::Mat rectified = rectify(frame, corners);

    // 正面化したフレームをデコード
    ZXing::ImageView iv(rectified.data, rectified.cols, rectified.rows, ZXing::ImageFormat::BGR,
                        static_cast<int>(rectified.step));

    auto qrCode = ZXing::ReadBarcode(iv, options);

    if(qrCode.isValid()) {
      QrCodeDetectionResult result;
      result.wasDetected = true;
      result.content = qrCode.text();
      for(int j = 0; j < 4; ++j) {
        result.corners[j] = corners[j];
      }
      results.push_back(result);
    }
  }

  return results;
}
