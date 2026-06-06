/**
 * @file   INetworkSystem.h
 * @brief  ネットワーク関数に対して依存性の注入を受ける基底クラス
 * @author sadomiya-sousi
 */

#pragma once
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class INetworkSystem {
 public:
  /**
   * @brief メモリリークを防ぐためにデストラクタを記述
   */
  virtual ~INetworkSystem() = default;

  /**
   * @brief socket()の抽象関数
   * @param domain 通信ドメイン
   * @param type ソケットタイプ
   * @param protocol
   */
  virtual int socket(int domain, int type, int protocol) = 0;
  /**
   * @brief close()の抽象関数
   * @param fd クローズするファイルディスクリプタ
   * @return 0:成功, -1:失敗
   */

  virtual int close(int fd) = 0;
  /**
   * @brief connect()の抽象関数
   * @param sockfd ソケットのファイルディスクリプタ
   * @param addr 接続先アドレス情報
   * @param addrlen アドレス構造体のサイズ
   * @return 0:成功, -1:失敗
   */
  virtual int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) = 0;

  /**
   * @brief send()の抽象関数
   * @param sockfd 送信先のファイルディスクリプタ
   * @param buf 送信データバッファのポインタ
   * @param len 送信データ長
   * @param flags 送信オプションのフラグ
   * @return 送信したバイト数, -1:失敗
   */
  virtual ssize_t send(int sockfd, const void* buf, size_t len, int flags) = 0;

  /**
   * @brief setsockopt()の抽象関数
   * @param sockfd 設定対象のファイルディスクリプタ
   * @param level プロトコルレベル
   * @param optname オプション名
   * @param optval 設定値のポインタ
   * @param optlen 設定値のバイト長
   * @return 0:成功, -1:失敗
   */
  virtual int setsockopt(int sockfd, int level, int optname, const void* optval, socklen_t optlen)
      = 0;

  /**
   * @brief bind()の抽象関数
   * @param sockfd ソケットのファイルディスクリプタ
   * @param addr 割り当てるアドレス情報
   * @param addrlen アドレス構造体のサイズ
   * @return 0:成功, -1:失敗
   */
  virtual int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) = 0;

  /**
   * @brief listen()の抽象関数
   * @param sockfd 待機状態にするファイルディスクリプタ
   * @param backlog 接続待ちキューの最大長
   * @return 0:成功, -1:失敗
   */
  virtual int listen(int sockfd, int backlog) = 0;

  /**
   * @brief accept()の抽象関数
   * @param sockfd 接続要求を待機しているファイルディスクリプタ
   * @param addr 接続元アドレスを格納する構造体
   * @param addrlen アドレス構造体のサイズへのポインタ
   * @return 新しい接続用のファイルディスクリプタ, -1:失敗
   */
  virtual int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) = 0;

  /**
   * @brief recv()の抽象関数
   * @param sockfd 受信対象のファイルディスクリプタ
   * @param buf 受信バッファのポインタ
   * @param len バッファの最大長
   * @param flags 受信オプションのフラグ
   * @return 受信したバイト数, 0:接続終了, -1:失敗
   */
  virtual ssize_t recv(int sockfd, void* buf, size_t len, int flags) = 0;
};