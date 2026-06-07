/**
 * @file   MockNetworkSystem.h
 * @brief  ネットワークライブラリのモッククラス
 * @author sadomiya-sousi
 */

#ifndef MOCK_NETWORK_SYSTEM_H
#define MOCK_NETWORK_SYSTEM_H

#include "INetworkSystem.h"
#include "SocketProtocol.h"
#include <cstring>
#include <cstdint>

class MockNetworkSystem : public INetworkSystem {
 public:
  bool forceSocketError = false;                     // socket() を失敗させるフラグ
  bool forceBindError = false;                       // bind() を失敗させるフラグ
  bool forceConnectError = false;                    // connect() を失敗させるフラグ
  bool receiveCommand = false;                       // recv() で1回だけ受信させるフラグ
  void* recvBuff = nullptr;                          // recv()で受け取るデータを格納
  int sizeOfReturnLen = CameraServer::COMMAND_SIZE;  // デフォルトでは1を返す
  uint8_t lastSentCommand = 0;                       // 送信したコマンドを保存する
  /**
   * @brief socket()のモック
   * @return ファイルディスクリプタとして999を返す
   */
  int socket(int domain, int type, int protocol) override
  {
    if(forceSocketError) return -1;  // 失敗
    return 999;                      // ダミーファイルディスクリプタ
  }

  /**
   * @brief setsoketopt()のモック
   * @param sockfd 設定対象のファイルディスクリプタ
   * @param level プロトコル対象の指定
   * @param optname オプションの種類
   * @param optval ポートの再利用のOnなどに応じた値へのポインタ
   * @param optlen optvalで渡した値のバイト長
   * @return 0:成功
   */
  int setsockopt(int sockfd, int level, int optname, const void* optval, socklen_t optlen) override
  {
    return 0;
  }

  /**
   * @brief bind()のモック
   * @param sock ポート番号と紐づけるファイルディスクリプタ
   * @param addr ポート番号等の入ったインターネットドメインソケットアドレス
   * @param addrlen インターネットドメインソケットアドレスのバイト長
   * @return 0:成功, -1:失敗
   */
  int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    if(forceBindError) return -1;  // 失敗
    return 0;                      // 成功
  }

  /**
   * @brief listen()のモック
   * @param sockfd 待機状態にするファイルディスクリプタ
   * @param backlog 接続待ち状態として保持できるキューの数
   * @return 0:成功
   */
  int listen(int sockfd, int backlog) override
  {
    return 0;  // 成功
  }

  /**
   * @brief return
   * @param sockfd 接続を確立するキューのファイルディスクリプタ
   * @param addr ポート番号等の入ったインターネットドメインソケットアドレス
   * @param addrlen インターネットドメインソケットアドレスのバイト長
   * @return 888:偽の通信用のファイルディスクリプタ
   */
  int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) override
  {
    return 888;  // ダミーのクライアントソケットを返す
  }

  /**
   * @brief connect()のモック
   * @param sockfd 通信を閉じるファイルディスクリプタ
   * @param addr インターネットドメインソケットアドレス
   * @param addrlen 構造体のバイト長
   * @return 0:成功, -1:失敗
   */
  int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    if(forceConnectError) return -1;  // 失敗
    return 0;                         // 成功
  }

  /**
   * @brief recv()のモック
   * @param sockfdデータを受け渡すファイルディスクリプタ。
   * サーバー側ならaccept()で発行したファイルディスクリプタでクライアント側ならsocket()で生成したファイルディスクリプタ
   * @param buf 受け取るバッファのポインタ
   * @param len バッファのバイト長
   * @param flags 受信オプションのフラグ
   * @return 成功:sizeOfReturnLenは操作可能な文字数, 失敗:-1
   */
  ssize_t recv(int sockfd, void* buf, size_t len, int flags) override
  {
    if(receiveCommand) {
      void* cmd = recvBuff;
      std::memcpy(buf, &cmd, sizeof(cmd));
      receiveCommand = false;
      return sizeOfReturnLen;
    }

    return -1;
  }

  /**
   * @brief send()のモック
   * @param sockfd 送信対象のファイルディスクリプタ
   * @param buf 送信するバッファのポインタ
   * @param addrlen バッファのバイト長
   * @return len 送信したバイト数
   */
  ssize_t send(int sockfd, const void* buf, size_t len, int flags) override
  {
    if(len > 0 && buf != nullptr) {
      lastSentCommand = *reinterpret_cast<const uint8_t*>(buf);
    }
    return len;
  }

  /**
   * @brief close()のモック
   * @param fd ファイルディスクリプタ
   * @return 0:成功
   */
  int close(int fd) override { return 0; }
};

#endif  // MOCK_NETWORK_SYSTEM_H
