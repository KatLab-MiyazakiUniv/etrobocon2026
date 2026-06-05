/**
 * @file QRCodeDetector.h
 * @brief 2次元コード検出用の画像処理クラス
 * @author yutaro-1214
 */

#ifndef QR_CODE_DETECTOR_H
#define QR_CODE_DETECTOR_H

#include "BoundingBoxDetector.h"
#include "SystemInfo.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include "Logger.h"

struct QRCodeDetectionResult {
  bool wasDetected = false;

  struct QRCodeInfo {
    std::string decodedText;

    cv::Point topLeft;
    cv::Point topRight;
    cv::Point bottomLeft;
    cv::Point bottomRight;
  };

  std::vector<QRCodeInfo> qrCodes;
};

class QRCodeDetector {
 public:
  QRCodeDetector();
  ~QRCodeDetector();

  void detect(const cv::Mat& frame, QRCodeDetectionResult& result);

 private:
  cv::QRCodeDetector qrDetector;
};

#endif