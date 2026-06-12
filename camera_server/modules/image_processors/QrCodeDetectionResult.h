/**
 * @file   QrCodeDetectionResult.h
 * @brief  QRコード検出結果のデータ構造定義
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTION_RESULT_H
#define QR_CODE_DETECTION_RESULT_H

#include <array>
#include <string>
#include <opencv2/core.hpp>

struct QrCodeDetectionResult {
  bool wasDetected = false;                 // 検出したかどうか
  std::string content;                      // QRコードから取得した文字列
  std::array<cv::Point2f, 4> corners = {};  // QRコードの各頂点の座標(左上から時計回りの順)
};

#endif  // QR_CODE_DETECTION_RESULT_H
