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
  bool wasDetected = false;               // 検出したかどうか
  std::string gatePosition;               // ゲート位置情報を表す文字列
  std::array<cv::Point, 4> corners = {};  // 検出したQRコードの各頂点の座標
};

#endif  // QR_CODE_DETECTION_RESULT_H
