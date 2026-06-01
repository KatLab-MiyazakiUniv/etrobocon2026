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
#include <iostream>

TEST(SocketServerTest, InitSuccessTest)
{
  MockNetworkSystem mockNet;
  SocketServer server(&mockNet);
  EXPECT_TRUE(server.init());
  server.run();
}
