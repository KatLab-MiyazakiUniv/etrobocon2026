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

  /**
   * @brief コマンド型のバイトサイズ
   */
  constexpr size_t COMMAND_SIZE =
      sizeof(Command);

  // =========================================================
  // Snapshot
  // =========================================================

  /**
   * @brief スナップショット撮影アクションのリクエストデータ構造
   */
  struct SnapshotActionRequest {
    Command command =
        Command::SNAPSHOT;

    /**
     * @brief 保存するファイル名
     */
    char fileName[64];
  };

  /**
   * @brief スナップショット撮影アクションのレスポンスデータ構造
   */
  struct SnapshotActionResponse {
    /**
     * @brief 撮影が成功したか
     */
    bool success;
  };

  // =========================================================
  // 共通データ構造
  // =========================================================

  /**
   * @brief HSVの3要素データ
   */
  struct ScalarData {
    double h = 0.0;
    double s = 0.0;
    double v = 0.0;
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
    int32_t width = 0;
    int32_t height = 0;
  };

  /**
   * @brief 座標データ
   */
  struct PointData {
    int32_t x = 0;
    int32_t y = 0;
  };

  /**
   * @brief バウンディングボックスを表す座標
   */
  struct BoundingBoxDetectionResult {
    /**
     * @brief 検出できたか
     */
    bool wasDetected = false;

    /**
     * @brief 左上
     */
    PointData topLeft;

    /**
     * @brief 右上
     */
    PointData topRight;

    /**
     * @brief 左下
     */
    PointData bottomLeft;

    /**
     * @brief 右下
     */
    PointData bottomRight;
  };

  // =========================================================
  // ColorRegionDetection
  // =========================================================

  /**
   * @brief 1リクエストで指定可能なHSV範囲の最大数
   */
  static constexpr uint32_t MAX_HSV_RANGES = 5;

  /**
   * @brief 1つの色に対応するHSV範囲
   */
  struct HSVRangeData {
    /**
     * @brief HSV下限値
     */
    ScalarData lower;

    /**
     * @brief HSV上限値
     */
    ScalarData upper;
  };

  /**
   * @brief カメラサーバーに色領域検出を要求するリクエスト構造体
   */
  struct ColorRegionDetectorRequest {
    /**
     * @brief 色領域検出コマンド
     */
    Command command =
        Command::COLOR_REGION_DETECTION;

    /**
     * @brief 最も大きい色領域のインデックスを返すか
     */
    bool requireLargestColorIndex = false;

    /**
     * @brief hsvRangesの有効要素数
     */
    uint8_t hsvRangeCount = 0;

    /**
     * @brief HSV範囲
     */
    HSVRangeData hsvRanges[MAX_HSV_RANGES];

    /**
     * @brief 検出対象ROI
     */
    RectData roi;
  };

  /**
   * @brief 色領域検出レスポンス
   */
  struct ColorRegionDetectorResponse {
    /**
     * @brief 色領域検出結果
     */
    BoundingBoxDetectionResult result;

    /**
     * @brief 最も面積が大きい色のインデックス
     */
    int32_t largestColorIndex = -1;
  };

  // =========================================================
  // QR Code Detection
  // =========================================================

  /**
   * @brief QRコードの頂点数
   */
  static constexpr uint32_t QR_CODE_CORNER_COUNT = 4;

  /**
   * @brief QRコードから取得した文字列の最大バイト数
   */
  static constexpr uint32_t QR_CODE_CONTENT_SIZE = 64;

  /**
   * @brief カメラサーバーにQRコード検出を要求するリクエスト構造体
   */
  struct QrCodeDetectorRequest {
    /**
     * @brief QRコード検出コマンド
     */
    Command command =
        Command::QR_CODE_DETECTION;

    /**
     * @brief 検出対象ROI
     */
    RectData roi;
  };

  /**
   * @brief QRコード検出レスポンス
   */
  struct QrCodeDetectorResponse {
    /**
     * @brief QRコードを検出できたか
     */
    bool wasDetected = false;

    /**
     * @brief QRコードの内容
     */
    char content[QR_CODE_CONTENT_SIZE] = {};

    /**
     * @brief QRコードの4頂点
     */
    PointData corners[QR_CODE_CORNER_COUNT] = {};
  };

  // =========================================================
  // Square Detection
  // =========================================================

  /**
   * @brief 正方形の頂点数
   */
  static constexpr uint32_t SQUARE_CORNER_COUNT = 4;

  /**
   * @brief カメラサーバーに正方形検出を要求するリクエスト構造体
   */
  struct SquareDetectorRequest {
    /**
     * @brief 正方形検出コマンド
     */
    Command command =
        Command::SQUARE_DETECTION;

    /**
     * @brief 検出対象ROI
     */
    RectData roi;

    /**
     * @brief SquareDetectorの追跡状態をリセットするか
     *
     * true:
     *   今回の正方形検出前に、
     *   previous detectionなどの追跡状態を破棄する。
     *
     * false:
     *   前回検出位置を引き継いで追跡する。
     *
     * Straightを挟んだ後の最初の正方形検出ではtrueを指定し、
     * 同じSquareAngleAdjustment内の2回目以降の検出では
     * falseを指定する。
     */
    bool resetTracking = false;
  };

  /**
   * @brief 正方形検出レスポンス
   */
  struct SquareDetectorResponse {
    /**
     * @brief 正方形を検出できたか
     */
    bool wasDetected = false;

    /**
     * @brief 正方形の4頂点
     */
    PointData corners[SQUARE_CORNER_COUNT] = {};
  };

}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H