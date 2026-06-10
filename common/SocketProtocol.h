/**
 * @file   SocketProtocol.h
 * @brief  Socket通信の共通定義ファイル
 * @author sadomiya-sousi takuchi17
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>
#include <cstddef>

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

  /**
   * @brief HSVやRGBAなどの4要素データ
   */
  struct ScalarData {
    double v0 = 0.0;  // 1要素目
    double v1 = 0.0;  // 2要素目
    double v2 = 0.0;  // 3要素目
    double v3 = 0.0;  // 4要素目
  };

  /**
   * @brief 矩形領域データ
   */
  struct RectData {
    int32_t x = 0;       // 左上のx座標
    int32_t y = 0;       // 左上のy座標
    int32_t width = 0;   // 幅
    int32_t height = 0;  // 高さ
  };

  /**
   * @brief 画像サイズデータ
   */
  struct SizeData {
    int32_t width = 0;   // 幅
    int32_t height = 0;  // 高さ
  };

  /**
   * @brief 座標データ
   */
  struct PointData {
    int32_t x = 0;  // x座標
    int32_t y = 0;  // y座標
  };

  /**
   * @brief バウンディングボックス検出結果
   */
  struct BoundingBoxDetectionResult {
    bool wasDetected = false;  // 検出できたかどうか

    PointData topLeft;      // 検出領域の左上の座標
    PointData topRight;     // 検出領域の右上の座標
    PointData bottomLeft;   // 検出領域の左下の座標
    PointData bottomRight;  // 検出領域の右下の座標
  };

  // ----- 以降、色領域検出のためのプロトコル定義 -----

  static constexpr uint32_t MAX_HSV_RANGES = 5;  // 1リクエストで指定可能なHSV範囲の最大数

  /**
   * @brief 1つの色に対応するHSVの範囲を表す構造体
   */
  struct HSVRangeData {
    ScalarData lower;  // HSVの下限値
    ScalarData upper;  // HSVの上限値
  };

  /**
   * @brief カメラサーバーに色領域検出を要求する際のリクエスト構造体
   */
  struct ColorRegionDetectorRequest {
    Command command = Command::COLOR_REGION_DETECTION;  // 色領域検出コマンド

    bool requireLargestColorIndex = false;   // 最も大きい色領域のインデックスを返すかどうか
    uint32_t hsvRangeCount = 0;              // hsvRangesの有効な要素数
    HSVRangeData hsvRanges[MAX_HSV_RANGES];  // HSVの範囲の配列

    RectData roi;         // 検出対象の領域
  };

  /**
   * @brief 色領域検出のレスポンス構造体
   */
  struct ColorRegionDetectorResponse {
    BoundingBoxDetectionResult result;  // 色領域の検出結果
    int32_t largestColorIndex = -1;     // 最も面積が大きい色のインデックス
  };

}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H