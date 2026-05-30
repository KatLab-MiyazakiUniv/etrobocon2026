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
    DISCONNECT = 254,  // クライアントからの切断要求
    SHUTDOWN = 255     // サーバーシャットダウン
  };
}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H