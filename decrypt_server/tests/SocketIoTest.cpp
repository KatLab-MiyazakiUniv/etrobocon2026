/**
 * @file   SocketIoTest.cpp
 * @brief  SocketIoクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "SocketIo.h"
#include "MockByteStreamNetworkSystem.h"
#include <cstdint>
#include <vector>

namespace etrobocon2026_test {

  // 1回のrecv()で全て受信できる場合、正しく受信できることを確認する
  TEST(SocketIoTest, RecvExactSucceedsAtOnce)
  {
    MockByteStreamNetworkSystem mockNet;
    uint8_t expected[4] = { 1, 2, 3, 4 };
    mockNet.pushRecvData(expected, sizeof(expected));

    uint8_t actual[4] = { 0 };
    EXPECT_TRUE(SocketIo::recvExact(mockNet, 888, actual, sizeof(actual)));
    for(size_t i = 0; i < sizeof(expected); i++) {
      EXPECT_EQ(expected[i], actual[i]);
    }
  }

  // recv()が1バイトずつしか返さない場合でも、全て受信しきることを確認する
  TEST(SocketIoTest, RecvExactSucceedsWithPartialReceive)
  {
    MockByteStreamNetworkSystem mockNet;
    uint8_t expected[8] = { 10, 20, 30, 40, 50, 60, 70, 80 };
    mockNet.pushRecvData(expected, sizeof(expected));
    mockNet.maxRecvChunk = 1;  // 部分受信を強制的に再現する

    uint8_t actual[8] = { 0 };
    EXPECT_TRUE(SocketIo::recvExact(mockNet, 888, actual, sizeof(actual)));
    for(size_t i = 0; i < sizeof(expected); i++) {
      EXPECT_EQ(expected[i], actual[i]);
    }
  }

  // 要求したバイト数に満たないうちに接続が切れた場合、falseを返すことを確認する
  TEST(SocketIoTest, RecvExactFailsOnDisconnect)
  {
    MockByteStreamNetworkSystem mockNet;
    uint8_t data[2] = { 1, 2 };
    mockNet.pushRecvData(data, sizeof(data));

    uint8_t actual[4] = { 0 };
    EXPECT_FALSE(SocketIo::recvExact(mockNet, 888, actual, sizeof(actual)));
  }

  // 受信データが無い場合、falseを返すことを確認する
  TEST(SocketIoTest, RecvExactFailsWhenNoData)
  {
    MockByteStreamNetworkSystem mockNet;
    uint8_t actual[4] = { 0 };
    EXPECT_FALSE(SocketIo::recvExact(mockNet, 888, actual, sizeof(actual)));
  }

  // 指定したバイト数を全て送信できることを確認する
  TEST(SocketIoTest, SendAllSucceeds)
  {
    MockByteStreamNetworkSystem mockNet;
    uint8_t expected[4] = { 9, 8, 7, 6 };

    EXPECT_TRUE(SocketIo::sendAll(mockNet, 888, expected, sizeof(expected)));
    ASSERT_EQ(sizeof(expected), mockNet.sentBytes.size());
    for(size_t i = 0; i < sizeof(expected); i++) {
      EXPECT_EQ(expected[i], mockNet.sentBytes[i]);
    }
  }

  // send()が失敗した場合、falseを返すことを確認する
  TEST(SocketIoTest, SendAllFailsOnSendError)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.forceSendError = true;
    uint8_t data[4] = { 1, 2, 3, 4 };

    EXPECT_FALSE(SocketIo::sendAll(mockNet, 888, data, sizeof(data)));
  }

}  // namespace etrobocon2026_test
