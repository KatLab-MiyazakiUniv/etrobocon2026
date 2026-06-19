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
    TAKE_SNAPSHOT = 1,       // スナップショット
    GET_DECRYPTION_KEY = 2,  // 復号キー取得
    DISCONNECT = 254,        // サーバーから切断
    SHUTDOWN = 255           // サーバーをシャットダウン
  };

  constexpr size_t COMMAND_SIZE = sizeof(Command);  // コマンド型のバイトサイズ

  /**
   * @brief HSVの4要素データ
   */
  struct ScalarData {
    double v0 = 0.0;  // h
    double v1 = 0.0;  // s
    double v2 = 0.0;  // v
    double v3 = 0.0;  // 透明度
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

  // スナップショット撮影アクションのリクエストデータ構造
  struct SnapshotActionRequest {
    Command command = Command::TAKE_SNAPSHOT;  // TAKE_SNAPSHOTを期待
    char fileName[64];                         // 保存するファイル名
  };

  // スナップショット撮影アクションのレスポンスデータ構造
  struct SnapshotActionResponse {
    bool success;  // 撮影が成功したかどうか
  };

  // 復号キー取得要求のリクエストデータ構造
  struct DecryptionKeyRequest {
    Command command = Command::GET_DECRYPTION_KEY;
  };

  // 復号キー取得要求のレスポンスデータ構造
  struct DecryptionKeyResponse {
    char key[5];
  };

}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H