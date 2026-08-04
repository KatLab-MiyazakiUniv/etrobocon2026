/**
 * @file   SocketIo.h
 * @brief  指定バイト数を送受信しきるまで繰り返すユーティリティクラス
 * @author HaruArima08
 */

#ifndef SOCKET_IO_H
#define SOCKET_IO_H

#include "INetworkSystem.h"
#include <cstddef>

class SocketIo {
 public:
  /**
   * @brief 指定バイト数を受信しきるまでrecv()を繰り返す
   * @param netSys 注入する具象クラス
   * @param sock ソケットファイルディスクリプタ
   * @param buf 受信バッファのポインタ
   * @param length 受信したいバイト数
   * @return true 指定バイト数を受信できた場合
   * @return false 接続断またはエラーが発生した場合
   */
  static bool recvExact(INetworkSystem& netSys, int sock, void* buf, size_t length)
  {
    size_t received = 0;
    char* dst = static_cast<char*>(buf);
    while(received < length) {
      ssize_t result = netSys.recv(sock, dst + received, length - received, 0);
      // 0は接続終了，負値は受信エラーを表す
      if(result <= 0) return false;
      received += static_cast<size_t>(result);
    }
    return true;
  }

  /**
   * @brief 指定バイト数を送信しきるまでsend()を繰り返す
   * @param netSys 注入する具象クラス
   * @param sock ソケットファイルディスクリプタ
   * @param buf 送信バッファのポインタ
   * @param length 送信したいバイト数
   * @return true 指定バイト数を送信できた場合
   * @return false 送信エラーが発生した場合
   */
  static bool sendAll(INetworkSystem& netSys, int sock, const void* buf, size_t length)
  {
    size_t sent = 0;
    const char* src = static_cast<const char*>(buf);
    while(sent < length) {
      ssize_t result = netSys.send(sock, src + sent, length - sent, 0);
      if(result <= 0) return false;
      sent += static_cast<size_t>(result);
    }
    return true;
  }

 private:
  SocketIo();  // インスタンス化禁止
};

#endif  // SOCKET_IO_H
