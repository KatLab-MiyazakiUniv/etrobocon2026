/**
 * @file SocketServerTest.cpp
 * @brief SocketServerクラスをテストする
 * @author sadomiya-sousi
 */

#include <iostream>
#include <unistd.h>
#include "SocketServer.h"
#include "MockNetworkSystem.h"
#include <gtest/gtest.h>

TEST(SocketServerTest, ConstructorSetsPortCorrectly)
{
  MockNetworkSystem mockNet;
  int testPort = 12345;
  SocketServer server(&mockNet, testPort);
  EXPECT_EQ(testPort, server.port);
}

TEST(SocketServerTest, InitSuccessTest)
{
  MockNetworkSystem mockNet;
  SocketServer server(&mockNet, 27015);
  EXPECT_TRUE(server.init());
  // server.run();
}
