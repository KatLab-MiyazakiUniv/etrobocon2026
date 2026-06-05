/*
 * @file QRCodeDetector.cpp
 * @brief 2次元コード検出用の画像処理クラス
 * @author yutaro-1214
 */

#include "QRCodeDetector.h"

// 全てのパラメータを個別に指定する
QRCodeDetector::QRCodeDetector()
{
  LOG_CREATE("QRCodeDetector");
}

QRCodeDetector::~QRCodeDetector()
{
  LOG_DESTROY("QRCodeDetector");
}

void QRCodeDetector::detect(const cv::Mat& frame, QRCodeDetectionResult& result)
{
  result.wasDetected = false;
  result.qrCodes.clear();

  if(frame.empty()) {
    Logger::error("入力フレームが空です。");
    return;
  }

  std::vector<cv::String> decodedTexts;
  std::vector<cv::Point> points;

  bool detected = qrDetector.detectAndDecodeMulti(frame, decodedTexts, points);

  if(!detected || decodedTexts.empty()) {
    Logger::error("QRコードが見つかりませんでした。");
    return;
  }

  result.wasDetected = true;

  for(size_t i = 0; i < decodedTexts.size(); i++) {
    size_t index = i * 4;

    if(index + 3 >= points.size()) {
      continue;
    }

    QRCodeDetectionResult::QRCodeInfo info;

    info.decodedText = decodedTexts[i];

    info.topLeft = points[index];
    info.topRight = points[index + 1];
    info.bottomRight = points[index + 2];
    info.bottomLeft = points[index + 3];

    result.qrCodes.push_back(info);
  }
}