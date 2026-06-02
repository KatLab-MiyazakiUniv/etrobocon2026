/**
 * @file   MockNetworkSystem.h
 * @brief  ネットワーク関数のモックを注入するクラス
 * @author sadomiya-sousi
 */
#pragma once
#include <cstring>
#include <vector>
#include "INetworkSystem.h"

class MockNetworkSystem : public INetworkSystem {
 public:
  bool forceConnectError = false;       // 関数の成功と失敗を制御
  std::vector<char> dummyResponseData;  // recv()で返すデータ
  /**
   * @brief socket()のモック
   * @return ファイルディスクリプタとして999を返す
   */
  int socket(int domain, int type, int protocol) override { return 999; }

  /**
   * @brief close()のモック
   * @param fd ファイルディスクリプタ
   * @return 0:成功
   */
  int close(int fd) override { return 0; }
  /**
   * @brief connect()のモック
   * @param sockfd 通信を閉じるファイルディスクリプタ
   * @param addr インターネットドメインソケットアドレス
   * @param addrlen 構造体のバイト長
   * @return 0:成功, 1:失敗
   */
  int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    return forceConnectError ? -1 : 0;
  }

  /**
   * @brief send()のモック
   * @param sockfd 送信対象のファイルディスクリプタ
   * @param buf 送信するバッファのポインタ
   * @param addrlen バッファのバイト長
   * @return len 送信したバイト数
   */
  ssize_t send(int sockfd, const void* buf, size_t len, int flags) override { return len; }

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
   * @param sock ポート番号と紐づけるファイルディスクリプタ
   * @param addr ポート番号等の入ったインターネットドメインソケットアドレス
   * @param addrlen インターネットドメインソケットアドレスのバイト長
   * @return 0:成功
   */
  int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override { return 0; }

  /**
   * @param sockfd 待機状態にするファイルディスクリプタ
   * @param backlog 接続待ち状態として保持できるキューの数
   * @param 0:成功
   */
  int listen(int sockfd, int backlog) override { return 0; }

  /**
   * @param sockfd 接続を確立するキューのファイルディスクリプタ
   * @param addr ポート番号等の入ったインターネットドメインソケットアドレス
   * @param addrlen インターネットドメインソケットアドレスのバイト長
   * @retrun 888:偽の通信用のファイルディスクリプタ
   */
  int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) override { return 888; }

  /**
   * @param sockfdデータを受け渡すファイルディスクリプタ。
   * サーバ側ならaccept()で発行したファイルディスクリプタでクライアント側ならsocket()で生成したファイルディスクリプタ
   * @param buf 受け取るバッファのポインタ
   * @param len バッファのバイト長
   * @param flgas 受信オプションのフラグ
   */
  ssize_t recv(int sockfd, void* buf, size_t len, int flags) override
  {
    if(dummyResponseData.empty()) return 0;  // データがなければ0を返す
    size_t copySize = std::min(len, dummyResponseData.size());
    std::memcpy(buf, dummyResponseData.data(), copySize);
    return copySize;
  }
};
