/**
 * @file   SocketProtocol.h
 * @brief  Socket通信の共通定義ファイル
 * @author sadomiya-sousi
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>
#include "RealNetworkSystem.h"

namespace CameraServer {

  static constexpr int DEFAULT_PORT = 27015;
  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    DISCONNECT = 254,  // サーバーから切断
    SHUTDOWN = 255     // サーバーをシャットダウン
  };
  constexpr size_t COMMAND_SIZE = sizeof(Command);
  inline RealNetworkSystem real;  // 注入するインスタンスの実体は1つ
}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H