/**
 * @file   SocketProtocol.h
 * @brief  Socket通信の共通定義ファイル
 * @author sadomiya-sousi takuchi17
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace CameraServer {

  /**
   * @brief カメラサーバーのデフォルトポート番号
   */
  static constexpr int DEFAULT_PORT = 27015;

  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    COLOR_REGION_DETECTION = 0,  // 色領域検出
    SNAPSHOT = 1,                // スナップショット
    QR_CODE_DETECTION = 2,       // QRコード検出
    SQUARE_DETECTION = 3,        // 正方形検出
    DISCONNECT = 254,            // サーバーから切断
    SHUTDOWN = 255               // サーバーをシャットダウン
  };

  constexpr size_t COMMAND_SIZE = sizeof(Command);  // コマンド型のバイトサイズ

  // スナップショット撮影アクションのリクエストデータ構造
  struct SnapshotActionRequest {
    Command command = Command::SNAPSHOT;  // SNAPSHOTを期待
    char fileName[64] = {};               // 保存するファイル名
  };

  // スナップショット撮影アクションのレスポンスデータ構造
  struct SnapshotActionResponse {
    bool success = false;  // 撮影成功かどうか
  };

  /**
   * @brief HSVの3要素データ
   */
  struct ScalarData {
    double h = 0.0;  // h
    double s = 0.0;  // s
    double v = 0.0;  // v
  };

  /**
   * @brief 矩形領域データ
   */
  struct RectData {
    int32_t x = 0;       // 左上X座標
    int32_t y = 0;       // 左上Y座標
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
    int32_t x = 0;  // X座標
    int32_t y = 0;  // Y座標
  };

  /**
   * @brief バウンディングボックスを表す座標
   */
  struct BoundingBoxDetectionResult {
    bool wasDetected = false;  // 検出できたか
    PointData topLeft;         // 左上の座標
    PointData topRight;        // 右上の座標
    PointData bottomLeft;      // 左下の座標
    PointData bottomRight;     // 右下の座標
  };

  static constexpr uint32_t MAX_HSV_RANGES = 5;  // 1リクエストで指定可能なHSV範囲の最大数

  /**
   * @brief 1つの色に対応するHSV範囲
   */
  struct HSVRangeData {
    ScalarData lower;  // HSV下限値
    ScalarData upper;  // HSV上限値
  };

  /**
   * @brief カメラサーバーに色領域検出を要求するリクエスト構造体
   */
  struct ColorRegionDetectorRequest {
    Command command = Command::COLOR_REGION_DETECTION;  // 色領域検出コマンド
    bool requireLargestColorIndex = false;   // 最も大きい色領域のインデックスを返すかどうか
    uint8_t hsvRangeCount = 0;               // hsvRangesの有効な要素数
    HSVRangeData hsvRanges[MAX_HSV_RANGES];  // HSVの範囲の配列
    RectData roi;                            // 検出対象の領域
  };

  /**
   * @brief 色領域検出のレスポンス構造体
   */
  struct ColorRegionDetectorResponse {
    BoundingBoxDetectionResult result;  // 色領域の検出結果
    int32_t largestColorIndex = -1;     // 最も面積が大きい色のインデックス
  };

  static constexpr uint32_t QR_CODE_CORNER_COUNT = 4;   // QRコードの頂点数
  static constexpr uint32_t QR_CODE_CONTENT_SIZE = 64;  // QRコードから取得した文字列の最大バイト数

  /**
   * @brief カメラサーバーにQRコード検出を要求する際のリクエスト構造体
   */
  struct QrCodeDetectorRequest {
    Command command = Command::QR_CODE_DETECTION;  // QRコード検出コマンド
    RectData roi;                                  // 検出対象の領域
  };

  /**
   * @brief QRコード検出のレスポンス構造体
   */
  struct QrCodeDetectorResponse {
    bool wasDetected = false;                      // 検出できたかどうか
    char content[QR_CODE_CONTENT_SIZE] = {};       // QRコードから取得した文字列
    PointData corners[QR_CODE_CORNER_COUNT] = {};  // QRコードの各頂点の座標(左上から時計回りの順)
  };

  static constexpr uint32_t SQUARE_CORNER_COUNT = 4;  // 短形の頂点数

  /**
   * @brief カメラサーバーに正方形検出を要求するリクエスト構造体
   */
  struct SquareDetectorRequest {
    Command command = Command::SQUARE_DETECTION;  // 短形検出コマンド
    RectData roi;                                 // 検出対象の領域
    bool resetTracking = false;                   // 前回の検出結果をリセットするかどうか
  };

  /**
   * @brief 正方形検出のレスポンス構造体
   */
  struct SquareDetectorResponse {
    bool wasDetected = false;                     // 検出できたかどうか
    PointData corners[SQUARE_CORNER_COUNT] = {};  // 短形の各頂点の座標
    double centerX = 0.0;                         // 画像上の正方形中心X座標[px]
    double centerY = 0.0;                         // 画像上の正方形中心Y座標[px]
    double forwardDistance = 0.0;                 // 正方形までの前方距離[mm]
    double lateralDistance = 0.0;                 // 正方形までの横方向距離[mm]
  };

}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H