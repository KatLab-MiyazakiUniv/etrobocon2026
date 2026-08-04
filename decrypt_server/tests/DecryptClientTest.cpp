/**
 * @file   DecryptClientTest.cpp
 * @brief  DecryptClientクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "DecryptClient.h"
#include "MockByteStreamNetworkSystem.h"
#include <cstring>

namespace etrobocon2026_test {

  // 生成直後の状態が想定通りであることを確認する
  TEST(DecryptClientTest, InitialState)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);

    int expected = -1;
    EXPECT_EQ(expected, client.getSock());
    EXPECT_FALSE(client.getIsConnected());
    EXPECT_EQ(DecryptServer::DEFAULT_PORT, client.getPort());
    EXPECT_EQ(std::string(DecryptServer::LOCAL_SERVER_IP), client.getServerIp());
  }

  // 正常に接続できることを確認する
  TEST(DecryptClientTest, ConnectSuccess)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);

    EXPECT_TRUE(client.connectToServer());
    EXPECT_TRUE(client.getIsConnected());
    EXPECT_TRUE(client.connectToServer());  // 接続済みの場合もtrueを返す
  }

  // socket()が失敗した場合、接続に失敗することを確認する
  TEST(DecryptClientTest, ConnectFailsOnSocketError)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.forceSocketError = true;
    DecryptClient client(mockNet);

    EXPECT_FALSE(client.connectToServer());
  }

  // 不正なIPアドレスを指定した場合、接続に失敗することを確認する
  TEST(DecryptClientTest, ConnectFailsOnInvalidIp)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet, DecryptServer::DEFAULT_PORT, "INVALID_IP_ADDRESS");

    EXPECT_FALSE(client.connectToServer());
  }

  // connect()が失敗した場合、接続に失敗することを確認する
  TEST(DecryptClientTest, ConnectFailsOnConnectError)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.forceConnectError = true;
    DecryptClient client(mockNet);

    EXPECT_FALSE(client.connectToServer());
  }

  // 切断時に復号システムの切断コマンドを送信することを確認する
  TEST(DecryptClientTest, DisconnectSendsDisconnectCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);
    client.connectToServer();

    client.disconnectFromServer();

    ASSERT_EQ(DecryptServer::COMMAND_SIZE, mockNet.sentBytes.size());
    uint8_t expected = static_cast<uint8_t>(DecryptServer::Command::DISCONNECT);
    EXPECT_EQ(expected, mockNet.sentBytes[0]);
    EXPECT_FALSE(client.getIsConnected());
  }

  // シャットダウン時に復号システムのシャットダウンコマンドを送信することを確認する
  TEST(DecryptClientTest, ShutdownSendsShutdownCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);
    client.connectToServer();

    client.shutdownServer();

    ASSERT_EQ(DecryptServer::COMMAND_SIZE, mockNet.sentBytes.size());
    uint8_t expected = static_cast<uint8_t>(DecryptServer::Command::SHUTDOWN);
    EXPECT_EQ(expected, mockNet.sentBytes[0]);
  }

  // 未接続の状態で切断を呼んでもクラッシュしないことを確認する
  TEST(DecryptClientTest, DisconnectWhileNotConnected)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);

    client.disconnectFromServer();

    size_t expected = 0;
    EXPECT_EQ(expected, mockNet.sentBytes.size());
  }

  // 未接続の状態で復号を要求した場合、falseを返すことを確認する
  TEST(DecryptClientTest, ExecuteDecryptFailsWhileNotConnected)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);

    DecryptServer::DecryptRequest request;
    DecryptServer::DecryptResponse response;
    EXPECT_FALSE(client.executeDecrypt(request, response));
  }

  // レスポンスを受信できた場合、内容が正しく取り出せることを確認する
  TEST(DecryptClientTest, ExecuteDecryptReceivesResponse)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);
    client.connectToServer();

    DecryptServer::DecryptResponse expectedResponse;
    expectedResponse.success = true;
    expectedResponse.decryptTimeMicro = 12345;
    expectedResponse.iterations = 3;
    mockNet.pushRecvData(&expectedResponse, sizeof(expectedResponse));

    DecryptServer::DecryptRequest request;
    DecryptServer::DecryptResponse response;
    EXPECT_TRUE(client.executeDecrypt(request, response));

    uint64_t expected = 12345;
    EXPECT_TRUE(response.success);
    EXPECT_EQ(expected, response.decryptTimeMicro);
    // リクエストが全長送信されていることを確認する
    EXPECT_EQ(sizeof(DecryptServer::DecryptRequest), mockNet.sentBytes.size());
  }

  // レスポンスが途中までしか届かなかった場合、falseを返すことを確認する
  TEST(DecryptClientTest, ExecuteDecryptFailsOnShortResponse)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);
    client.connectToServer();

    DecryptServer::DecryptResponse partialResponse;
    mockNet.pushRecvData(&partialResponse, sizeof(partialResponse) - 1);

    DecryptServer::DecryptRequest request;
    DecryptServer::DecryptResponse response;
    EXPECT_FALSE(client.executeDecrypt(request, response));
  }

  // Pingのレスポンスを受信できることを確認する
  TEST(DecryptClientTest, ExecutePingReceivesResponse)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);
    client.connectToServer();

    DecryptServer::PingResponse expectedResponse;
    mockNet.pushRecvData(&expectedResponse, sizeof(expectedResponse));

    DecryptServer::PingRequest request;
    DecryptServer::PingResponse response;
    EXPECT_TRUE(client.executePing(request, response));
    EXPECT_TRUE(response.success);
  }

  // ポート番号とIPアドレスを設定できることを確認する
  TEST(DecryptClientTest, SetPortAndServerIp)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptClient client(mockNet);

    int expectedPort = 30000;
    std::string expectedIp = "192.168.0.1";
    client.setPort(expectedPort);
    client.setServerIp(expectedIp);

    EXPECT_EQ(expectedPort, client.getPort());
    EXPECT_EQ(expectedIp, client.getServerIp());
  }

}  // namespace etrobocon2026_test
