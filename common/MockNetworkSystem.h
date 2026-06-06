#ifndef MOCK_NETWORK_SYSTEM_H
#define MOCK_NETWORK_SYSTEM_H

#include "INetworkSystem.h"
#include "SocketProtocol.h"  // CameraServer::Command を使うため
#include <cstring>
#include <cstdint>

class MockNetworkSystem : public INetworkSystem {
 public:
  bool forceSocketError = false;                     // socket() を失敗させるフラグ
  bool forceBindError = false;                       // bind() を失敗させるフラグ
  bool forceConnectError = false;                    // connect() を失敗させるフラグ
  bool receiveCommand = false;                       // recv() で1回だけ受信させるフラグ
  void* recvBuff = nullptr;                          // recv()で受け取るデータを格納する
  int sizeOfReturnLen = CameraServer::COMMAND_SIZE;  // デフォルトでは1を返す

  uint8_t lastSentCommand = 0;  // send() で送信されたデータを記録する箱

  // socket() の手動モック
  int socket(int domain, int type, int protocol) override
  {
    if(forceSocketError) return -1;  // 失敗
    return 999;                      // ダミーファイルディスクリプタ
  }

  // setsockopt() の手動モック
  int setsockopt(int sockfd, int level, int optname, const void* optval, socklen_t optlen) override
  {
    return 0;  // 常に成功
  }

  // bind() の手動モック
  int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    if(forceBindError) return -1;  // 失敗
    return 0;                      // 成功
  }

  // listen() の手動モック
  int listen(int sockfd, int backlog) override
  {
    return 0;  // 成功
  }

  // accept() の手動モック
  int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) override
  {
    return 888;  // ダミーのクライアントソケットを返す
  }

  // connect() の手動モック (クライアント用)
  int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    if(forceConnectError) return -1;  // 失敗
    return 0;                         // 成功
  }

  // recv() のモック (サーバー用)
  ssize_t recv(int sockfd, void* buf, size_t len, int flags) override
  {
    if(receiveCommand) {
      void* cmd = recvBuff;
      std::memcpy(buf, &cmd, sizeof(cmd));
      receiveCommand = false;
      return sizeOfReturnLen;  // コピーしたバイト数（1）を返す
    }

    return 0;
  }

  // send() のモック (クライアント用)
  ssize_t send(int sockfd, const void* buf, size_t len, int flags) override
  {
    if(len > 0 && buf != nullptr) {
      lastSentCommand = *reinterpret_cast<const uint8_t*>(buf);
    }
    return len;
  }

  // close() の手動モック
  int close(int fd) override
  {
    return 0;  // 成功
  }
};

#endif  // MOCK_NETWORK_SYSTEM_H
