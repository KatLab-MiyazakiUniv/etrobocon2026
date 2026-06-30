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
    SNAPSHOT = 1,      // スナップショット
    DISCONNECT = 254,  // サーバーから切断
    SHUTDOWN = 255     // サーバーをシャットダウン
  };

  constexpr size_t COMMAND_SIZE = sizeof(Command);  // コマンド型のバイトサイズ

  // スナップショット撮影アクションのリクエストデータ構造
  struct SnapshotActionRequest {
    Command command = Command::SNAPSHOT;  // SNAPSHOTを期待
    char fileName[64];                    // 保存するファイル名
  };

  // スナップショット撮影アクションのレスポンスデータ構造
  struct SnapshotActionResponse {
    bool success;  // 撮影が成功したかどうか
  };
}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H