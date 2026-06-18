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

  // executeColorRegionDetection の正常系テスト
  TEST(SocketClientTest, ExecuteColorRegionDetectionSuccess)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    EXPECT_TRUE(client.connectToServer());

    CameraServer::ColorRegionDetectorRequest request;
    request.requireLargestColorIndex = true;
    request.hsvRangeCount = 1;

    mockNet.hasRecvData = true;
    mockNet.sizeOfReturnLen = sizeof(CameraServer::ColorRegionDetectorResponse);

    CameraServer::ColorRegionDetectorResponse actualResponse;
    EXPECT_TRUE(client.executeColorRegionDetection(request, actualResponse));
    uint8_t expectedCmd = static_cast<uint8_t>(CameraServer::Command::COLOR_REGION_DETECTION);
    EXPECT_EQ(mockNet.lastSentCommand, expectedCmd);
  }

  // executeActionが成功しtrueを返すことを確認するテスト
  TEST(SocketClientTest, ExecuteActionSuccess)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    EXPECT_TRUE(client.connectToServer());

    CameraServer::ColorRegionDetectorRequest request;
    mockNet.hasRecvData = true;
    mockNet.sizeOfReturnLen = sizeof(CameraServer::ColorRegionDetectorResponse);

    CameraServer::ColorRegionDetectorResponse actualResponse;
    EXPECT_TRUE(client.executeAction(request, actualResponse));
  }

  // executeAction 未接続状態のテスト
  TEST(SocketClientTest, ExecuteActionNotConnected)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);

    CameraServer::ColorRegionDetectorRequest request;
    CameraServer::ColorRegionDetectorResponse response;
    EXPECT_FALSE(client.executeAction(request, response));
  }

  // executeAction 受信失敗テスト
  TEST(SocketClientTest, ExecuteActionRecvFailure)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    EXPECT_TRUE(client.connectToServer());
    mockNet.hasRecvData = false;
    CameraServer::ColorRegionDetectorRequest request;
    CameraServer::ColorRegionDetectorResponse response;
    EXPECT_FALSE(client.executeAction(request, response));
  }

  // executeAction 不完全なレスポンスのテスト
  TEST(SocketClientTest, ExecuteActionIncompleteResponse)
  {
    MockNetworkSystem mockNet;
    SocketClient client(mockNet);
    EXPECT_TRUE(client.connectToServer());

    // 期待するサイズより小さいサイズが受信されたと設定
    mockNet.hasRecvData = true;
    mockNet.sizeOfReturnLen = sizeof(CameraServer::ColorRegionDetectorResponse) - 1;

    CameraServer::ColorRegionDetectorRequest request;
    CameraServer::ColorRegionDetectorResponse response;
    EXPECT_FALSE(client.executeAction(request, response));
  }
}  // namespace etrobocon2026_test
