/**
 * @file   QrCodeDetector.cpp
 * @brief  2次元コード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector(bool _isCurved) : isCurved(_isCurved)
{
  detector.setEpsX(0.2); 
  detector.setEpsY(0.2);
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

  std::vector<cv::Point2f> cornerPoints;
  std::string gatePos;

  if(isCurved) {
    gatePos = detector.detectAndDecodeCurved(frame, cornerPoints);
  } else {
    gatePos = detector.detectAndDecode(frame, cornerPoints);
  }

  if(!gatePos.empty() && cornerPoints.size() >= 4) {
    result.wasDetected = true;
    result.gatePosition = gatePos;
    for(int i = 0; i < 4; ++i) {
      result.corners[i]
          = cv::Point(static_cast<int>(cornerPoints[i].x), static_cast<int>(cornerPoints[i].y));
    }
  }

  return result;
}
