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
  // 依存性の注入後にメモリリークを防ぐためにデストラクタを記述
  virtual ~INetworkSystem() = default;

  /**
   * @brief socket()のラッパー
   * @param domain 通信ドメイン
   * @param type ソケットタイプ
   * @param protocol
   */
  virtual int socket(int domain, int type, int protocol) = 0;
  virtual int close(int fd) = 0;
  virtual int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) = 0;
  virtual ssize_t send(int sockfd, const void* buf, size_t len, int flags) = 0;
  virtual int setsockopt(int sockfd, int level, int optname, const void* optval, socklen_t optlen)
      = 0;
  virtual int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) = 0;
  virtual int listen(int sockfd, int backlog) = 0;
  virtual int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) = 0;
  virtual ssize_t recv(int sockfd, void* buf, size_t len, int flags) = 0;
};