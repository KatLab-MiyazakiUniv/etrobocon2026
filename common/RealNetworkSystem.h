/**
 * @file   RealNetworkSystem.h
 * @brief  実際のネットワーク関数を注入するクラス
 * @author sadomiya-sousi
 */

#pragma once
#include "INetworkSystem.h"

namespace etrobocon2026 {

  class RealNetworkSystem : public INetworkSystem {
   public:
    int socket(int domain, int type, int protocol) override
    {
      return ::socket(domain, type, protocol);  //  :: でグローバル空間の関数を指定
    }
    int close(int fd) override { return ::close(fd); }
    int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
    {
      return ::connect(sockfd, addr, addrlen);
    }
    ssize_t send(int sockfd, const void* buf, size_t len, int flags) override
    {
      return ::send(sockfd, buf, len, flags);
    }
    int setsockopt(int sockfd, int level, int optname, const void* optval,
                   socklen_t optlen) override
    {
      return ::setsockopt(sockfd, level, optname, optval, optlen);
    }
    int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
    {
      return ::bind(sockfd, addr, addrlen);
    }
    int listen(int sockfd, int backlog) override { return ::listen(sockfd, backlog); }
    int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) override
    {
      return ::accept(sockfd, addr, addrlen);
    }
    ssize_t recv(int sockfd, void* buf, size_t len, int flags) override
    {
      return ::recv(sockfd, buf, len, flags);
    }
  };

}  // namespace etrobocon2026