/**
 * @file SocketClientTest.cpp
 * @brief SocketClientクラスをテストする
 * @author sadomiya-sousi, takuchi17
 */

#include <gtest/gtest.h>
#include "SocketClient.h"
#include "MockNetworkSystem.h"
#include "Logger.h"

namespace etrobocon2026_test {

  // 正常に接続できることを確認
  TEST(SocketClientTest, ConnectSuccess)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    EXPECT_TRUE(client.connectToServer());
    EXPECT_TRUE(client.connectToServer());
  }

  // socket() が失敗した場合、false を返すことを確認
  TEST(SocketClientTest, ConnectFailsOnSocketError)
  {
    MockNetworkSystem mockNet;
    mockNet.forceSocketError = true;  // socket() で -1 を返すように指示
    SocketClient client(mockNet);
    EXPECT_FALSE(client.connectToServer());
  }

  // htons() が失敗した場合、false を返すことを確認
  TEST(SocketClientTest, DISABLED_ConnectFailsOnInvalidPort)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet, -1);
    EXPECT_FALSE(client.connectToServer());
  }

  // inet_pton() が失敗した場合false を返すことを確認
  TEST(SocketClientTest, ConnectFailsOnInvalidIp)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet, 27015, "INVALID_IP_ADDRESS");
    EXPECT_FALSE(client.connectToServer());
  }

  // connect() が失敗した場合、false を返すことを確認
  TEST(SocketClientTest, ConnectFailsOnConnectError)
  {
    MockNetworkSystem mockNet;
    mockNet.forceConnectError = true;
    SocketClient client(mockNet);
    EXPECT_FALSE(client.connectToServer());
  }

  // 未接続の状態で切断を呼んでもクラッシュしないことを確認
  TEST(SocketClientTest, DisconnectWhileNotConnected)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    client.disconnectFromServer();
    SUCCEED();
  }

  // 接続後に切断すると、DISCONNECTコマンドが送信されることを確認
  TEST(SocketClientTest, DisconnectSendsCommand)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    client.connectToServer();
    client.disconnectFromServer();
    uint8_t expectedCmd = static_cast<uint8_t>(CameraServer::Command::DISCONNECT);
    EXPECT_EQ(mockNet.lastSentCommand, expectedCmd);
  }

  // 接続せずにshutdownコマンドを送ってもクラッシュしないことを確認
  TEST(SocketClientTest, ShutdownWhileNotConnected)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    client.shutdownServer();
    SUCCEED();
  }

  // 接続後にシャットダウンすると、SHUTDOWNコマンドが送信されることを確認
  TEST(SocketClientTest, ShutdownSendsCommand)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    client.connectToServer();
    client.shutdownServer();
    uint8_t expectedCmd = static_cast<uint8_t>(CameraServer::Command::SHUTDOWN);
    EXPECT_EQ(mockNet.lastSentCommand, expectedCmd);
  }
}  // namespace etrobocon2026_test
