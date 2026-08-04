/**
 * @file   MockByteStreamNetworkSystem.h
 * @brief  任意のバイト列を分割して受信できるネットワークライブラリのモッククラス
 * @author HaruArima08
 */

#ifndef MOCK_BYTE_STREAM_NETWORK_SYSTEM_H
#define MOCK_BYTE_STREAM_NETWORK_SYSTEM_H

#include "INetworkSystem.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

/**
 * @brief 任意のバイト列を分割して受信できるモッククラス
 * @note maxRecvChunkを1にすると、TCPのセグメント分割による部分受信を強制的に再現できる
 */
class MockByteStreamNetworkSystem : public INetworkSystem {
 public:
  bool forceSocketError = false;   // socket()を失敗させるフラグ
  bool forceBindError = false;     // bind()を失敗させるフラグ
  bool forceListenError = false;   // listen()を失敗させるフラグ
  bool forceConnectError = false;  // connect()を失敗させるフラグ
  bool forceSendError = false;     // send()を失敗させるフラグ
  std::vector<uint8_t> recvQueue;  // recv()で返すバイト列
  std::vector<uint8_t> sentBytes;  // send()で送信されたバイト列
  size_t maxRecvChunk = 0;         // 1回のrecv()で返す最大バイト数(0なら要求された全長)
  size_t recvOffset = 0;           // recvQueueの読み出し位置

  /**
   * @brief 受信させたいデータをキューへ追加する
   * @param data 追加するデータの先頭ポインタ
   * @param length 追加するデータのバイト長
   */
  void pushRecvData(const void* data, size_t length)
  {
    const uint8_t* src = static_cast<const uint8_t*>(data);
    recvQueue.insert(recvQueue.end(), src, src + length);
  }

  /**
   * @brief socket()のモック
   * @return 999:成功, -1:失敗
   */
  int socket(int domain, int type, int protocol) override
  {
    if(forceSocketError) return -1;
    return 999;
  }

  /**
   * @brief setsockopt()のモック
   * @return 0:成功
   */
  int setsockopt(int sockfd, int level, int optname, const void* optval, socklen_t optlen) override
  {
    return 0;
  }

  /**
   * @brief bind()のモック
   * @return 0:成功, -1:失敗
   */
  int bind(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    if(forceBindError) return -1;
    return 0;
  }

  /**
   * @brief listen()のモック
   * @return 0:成功, -1:失敗
   */
  int listen(int sockfd, int backlog) override
  {
    if(forceListenError) return -1;
    return 0;
  }

  /**
   * @brief accept()のモック
   * @return 888:ダミーのクライアントソケット
   */
  int accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) override { return 888; }

  /**
   * @brief connect()のモック
   * @return 0:成功, -1:失敗
   */
  int connect(int sockfd, const struct sockaddr* addr, socklen_t addrlen) override
  {
    if(forceConnectError) return -1;
    return 0;
  }

  /**
   * @brief recv()のモック
   * @return 読み出したバイト数, 0:キューが空
   */
  ssize_t recv(int sockfd, void* buf, size_t len, int flags) override
  {
    size_t remaining = recvQueue.size() - recvOffset;
    if(remaining == 0) return 0;  // 接続終了を表す

    size_t readLength = std::min(len, remaining);
    if(maxRecvChunk > 0) readLength = std::min(readLength, maxRecvChunk);

    std::memcpy(buf, recvQueue.data() + recvOffset, readLength);
    recvOffset += readLength;
    return static_cast<ssize_t>(readLength);
  }

  /**
   * @brief send()のモック
   * @return 送信したバイト数, -1:失敗
   */
  ssize_t send(int sockfd, const void* buf, size_t len, int flags) override
  {
    if(forceSendError) return -1;
    const uint8_t* src = static_cast<const uint8_t*>(buf);
    sentBytes.insert(sentBytes.end(), src, src + len);
    return static_cast<ssize_t>(len);
  }

  /**
   * @brief close()のモック
   * @return 0:成功
   */
  int close(int fd) override { return 0; }
};

#endif  // MOCK_BYTE_STREAM_NETWORK_SYSTEM_H
