/**
 * @file SocketClientTest.cpp
 * @brief SocketClientクラスをテストする
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include <iostream>
#include "SocketClient.h"
#include "MockNetworkSystem.h"
#include "Logger.h"

TEST(SocketClient, ConnectTest)
{
  MockNetworkSystem mockNet;
  mockNet.forceConnectError = false;
  SocketClient client = SocketClient(&mockNet, 27015, "127.0.0.1");
  if(client.connectToServer()) {
    client.disconnectFromServer();
  }
  // else {
  //   Logger::printfLog(Logger::ERROR, "サーバーが見つかりません");
  // }
}

// connectToServer()が呼ばれる前にdisconnectFromServer()を呼んでも問題ないことを確認
TEST(SocketClient, DisconnectBeforeConnect)
{
  MockNetworkSystem mockNet;
  SocketClient client = SocketClient(&mockNet, 27015, "127.0.0.1");
  client.disconnectFromServer();
  SUCCEED();
}

TEST(SocketClient, ConnectAndDisconnect)
{
  MockNetworkSystem mockNet;
  mockNet.forceConnectError = false;
  SocketClient client = SocketClient(&mockNet, 27015, "127.0.0.1");
  EXPECT_TRUE(client.connectToServer());
  client.disconnectFromServer();
  SUCCEED();
}
