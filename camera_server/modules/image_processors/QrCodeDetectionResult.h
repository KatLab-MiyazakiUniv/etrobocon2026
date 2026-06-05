/**
 * @file   QrCodeDetectionResult.h
 * @brief  2次元コード検出結果のデータ構造定義
 * @author HaruArima08
 */

#ifndef QR_CODE_DETECTION_RESULT_H
#define QR_CODE_DETECTION_RESULT_H

#include <array>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

// 2次元コードの情報を保持する構造体
struct QrCodeData {
  std::string gatePosition;               // ゲート位置情報を表す文字列
  std::array<cv::Point, 4> corners = {};  // 検出した2次元コードの各頂点の座標
};

// 2次元コード検出結果を保持する構造体
struct QrCodeDetectionResult {
  bool wasDetected = false;         // 検出したかどうか
  int count = 0;                    // 検出した2次元コードの個数
  std::vector<QrCodeData> qrCodes;  // 検出した2次元コードの情報を格納するベクター
};

#endif  // QR_CODE_DETECTION_RESULT_H
