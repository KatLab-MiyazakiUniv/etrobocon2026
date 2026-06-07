/**
 * @file   SocketProtocol.h
 * @brief  Socket通信の共通定義ファイル
 * @author sadomiya-sousi takuchi17
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>
#include <opencv2/core/types.hpp>

namespace CameraServer {

  static constexpr int DEFAULT_PORT = 27015;  // カメラサーバーのデフォルトのポート番号

  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    COLOR_REGION_DETECTION = 0,  // 色領域検出
    DISCONNECT = 254,            // サーバーから切断
    SHUTDOWN = 255               // サーバーをシャットダウン
  };

  constexpr size_t COMMAND_SIZE = sizeof(Command);  // コマンド型のバイトサイズ

  // バウンディングボックス検出結果
  struct BoundingBoxDetectionResult {
    bool wasDetected = false;  // 検出できたかどうか
    cv::Point topLeft;         // 検出領域の左上の座標
    cv::Point topRight;        // 検出領域の右上の座標
    cv::Point bottomLeft;      // 検出領域の左下の座標
    cv::Point bottomRight;     // 検出領域の右下の座標
  };

  // ----- 以降、色領域検出のためのプロトコル定義 -----

  static constexpr uint32_t MAX_HSV_RANGES = 5;  // 1リクエストで指定可能なHSV範囲の最大数

  // 1つの色に対応するHSVの範囲を表す構造体
  struct HSVRangeData {
    cv::Scalar lower;  // HSVの下限値 (H, S, V)
    cv::Scalar upper;  // HSVの上限値 (H, S, V)
  };

  // カメラサーバーに色領域検出を要求する際のリクエスト構造体
  struct ColorRegionDetectorRequest {
    Command command = Command::COLOR_REGION_DETECTION;  // 色領域検出コマンド
    bool requireLargestColorIndex = false;   // 最も大きい色領域のインデックスを返すかどうか
    uint32_t hsvRangeCount = 0;              // hsvRangesの要素数
    HSVRangeData hsvRanges[MAX_HSV_RANGES];  // HSVの範囲の配列
    cv::Rect roi;                            // 検出対象の領域
    cv::Size resolution;                     // 解像度
  };

  // 色領域検出のレスポンス構造体
  struct ColorRegionDetectorResponse {
    BoundingBoxDetectionResult result;  // 色領域の検出結果
    int32_t largestColorIndex = -1;     // 最も面積が大きい色のインデックス
  };
}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H