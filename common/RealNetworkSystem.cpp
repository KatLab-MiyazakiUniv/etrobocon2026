/**
 * @file   RealNetworkSystem.cpp
 * @brief  本物のネットワークライブラリのラッパークラス
 * @author sadomiya-sousi
 */

#include "RealNetworkSystem.h"

int RealNetworkSystem::socket(int domain, int type, int protocol)
{
  return ::socket(domain, type, protocol);
}

int RealNetworkSystem::close(int fd)
{
  return ::close(fd);
}

int RealNetworkSystem::connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen)
{
  return ::connect(sockfd, addr, addrlen);
}

ssize_t RealNetworkSystem::send(int sockfd, const void* buf, size_t len, int flags)
{
  return ::send(sockfd, buf, len, flags);
}

int RealNetworkSystem::setsockopt(int sockfd, int level, int optname, const void* optval,
                                  socklen_t optlen)
{
  return ::setsockopt(sockfd, level, optname, optval, optlen);
}

int RealNetworkSystem::bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen)
{
  return ::bind(sockfd, addr, addrlen);
}

int RealNetworkSystem::listen(int sockfd, int backlog)
{
  return ::listen(sockfd, backlog);
}

int RealNetworkSystem::accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen)
{
  return ::accept(sockfd, addr, addrlen);
}

ssize_t RealNetworkSystem::recv(int sockfd, void* buf, size_t len, int flags)
{
  return ::recv(sockfd, buf, len, flags);
}