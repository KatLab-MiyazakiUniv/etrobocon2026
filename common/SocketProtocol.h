/**
 * @file   SocketProtocol.h
 * @brief  Socket通信の共通定義ファイル
 * @author sadomiya-sousi, takuchi17
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>

namespace CameraServer {

  static constexpr int DEFAULT_PORT = 27015;  // カメラサーバーのデフォルトのポート番号

  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    DISCONNECT = 254,  // サーバーから切断
    SHUTDOWN = 255     // サーバーをシャットダウン
  };

  constexpr size_t COMMAND_SIZE = sizeof(Command);  // コマンド型のバイトサイズ
}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H