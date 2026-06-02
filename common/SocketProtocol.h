/**
 * @file   SocketProtocol.h
 * @brief  Socket通信の共通定義ファイル
 * @author sadomiya-sousi
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>
#include "RealNetworkSystem.h"
#include "MockNetworkSystem.h"

namespace CameraServer {
  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    DISCONNECT = 254,  // サーバから切断
    SHUTDOWN = 255     // サーバーをシャットダウン
  };
}  // namespace CameraServer

// 注入するインスタンスの実体は1つ
inline RealNetworkSystem real;

#endif  // SOCKET_PROTOCOL_H