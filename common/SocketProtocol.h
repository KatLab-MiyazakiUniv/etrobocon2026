/**
 * @file   SocketProtocol.h
 * @brief  カメラサーバーとの通信プロトコル定義
 * @author sadomiya-sousi
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>

namespace CameraServer {
  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    DISCONNECT = 254,  // サーバから切断
    SHUTDOWN = 255     // サーバーをシャットダウン
  };
}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H