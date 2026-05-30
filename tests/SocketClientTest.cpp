/**
 * @file SocketClientTest.cpp
 * @brief SocketClientクラスをテストする
 * @author sadomiya-sousi
 */

#include <iostream>
#include <unistd.h>

#include "EtRobocon2026.h"
#include "SocketClient.h"
#include "MockNetworkSystem.h"
#include <gtest/gtest.h>
#include <iostream>

TEST(SocketClient, ConnectTest)
{
  MockNetworkSystem mockNet;
  mockNet.forceConnectError = false;
  SocketClient client = SocketClient(&mockNet);
  if(client.connectToServer()) {
    client.disconnectFromServer();
  } else {
    std::cerr << "サーバーが見つかりません。サーバーが起動しているか確認してください。"
              << std::endl;
  }
}