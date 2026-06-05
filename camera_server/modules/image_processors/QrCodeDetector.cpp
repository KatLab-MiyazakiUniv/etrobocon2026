/**
 * @file   QrCodeDetector.cpp
 * @brief  2次元コード検出処理クラス
 * @author HaruArima08
 */

#include "QrCodeDetector.h"

QrCodeDetector::QrCodeDetector()
{
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

  std::vector<std::string> gatePositions;  // ゲート位置情報を格納するベクター
  std::vector<cv::Point> cornerPoints;     // コードの各頂点の座標を格納するベクター

  bool detected = detector.detectAndDecodeMulti(frame, gatePositions, cornerPoints);

  if(!detected) return result;

  for(size_t i = 0; i < gatePositions.size(); ++i) {
    if(cornerPoints.size() < (i + 1) * 4) {
      Logger::printfLog(Logger::WARNING,
                        "QrCodeDetector: コード[%zu]の頂点座標が取得できませんでした。", i);
      continue;
    }
    QrCodeData data;
    data.gatePosition = gatePositions[i];
    data.corners = { cornerPoints[i * 4 + 0], cornerPoints[i * 4 + 1], cornerPoints[i * 4 + 2],
                     cornerPoints[i * 4 + 3] };
    result.qrCodes.push_back(data);
  }

  result.count = static_cast<int>(result.qrCodes.size());
  result.wasDetected = !result.qrCodes.empty();

  return result;
}
