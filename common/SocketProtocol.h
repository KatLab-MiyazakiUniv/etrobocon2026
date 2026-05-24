/**
 * @file   SocketProtocol.h
 * @brief  カメラサーバーとの通信プロトコル定義
 * @author okuyama0528, sadomiya-sousi
 */

#ifndef SOCKET_PROTOCOL_H
#define SOCKET_PROTOCOL_H

#include <cstdint>

namespace CameraServer {

  /**
   * @brief サーバー側で実行可能なコマンド
   */
  enum class Command : uint8_t {
    SHUTDOWN = 255  // サーバーシャットダウン
  };

  /**
   * @brief シャットダウンコマンド（送信専用）
   */
  struct ShutdownRequest {
    Command command = Command::SHUTDOWN;
  };

}  // namespace CameraServer

#endif  // SOCKET_PROTOCOL_H