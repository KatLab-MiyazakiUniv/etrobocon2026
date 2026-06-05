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

// QRコードの情報を保持する構造体
struct QRCodeDetectionResult {
  // QRコードがあるかどうか
  bool wasDetected = false;
  // 一つのQRコードの情報
  struct QRCodeInfo {
    // 文字列の情報
    std::string decodedText;
    // QRコードの四隅の座標
    cv::Point topLeft;
    cv::Point topRight;
    cv::Point bottomLeft;
    cv::Point bottomRight;
  };
  // QRコードが複数あるか
  std::vector<QRCodeInfo> qrCodes;
};

class QRCodeDetector {
 public:
  /**
   * @brief コンストラクタ
   */
  QRCodeDetector();

  /**
   * @brief デストラクタ
   */
  ~QRCodeDetector();

  /**
   * @brief フレームからQRコードを検出
   * @param frame 入力フレーム
   * @param result QRコードの情報
   */
  void detect(const cv::Mat& frame, QRCodeDetectionResult& result);

 private:
  cv::QRCodeDetector qrDetector;
};

#endif