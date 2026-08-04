/**
 * @file   DecryptSocketServerTest.cpp
 * @brief  DecryptSocketServerクラスをテストする
 * @author HaruArima08
 */

#include <gtest/gtest.h>
#include "DecryptSocketServer.h"
#include "MockByteStreamNetworkSystem.h"
#include <cstring>
#include <string>

namespace etrobocon2026_test {

  // 平文"Hello"を鍵"1234"で暗号化した文
  static const char* SERVER_TEST_ENCRYPTED_TEXT = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";

  /**
   * @brief 復号リクエストを組み立てる
   * @param key 復号キー
   * @param encryptedText 暗号文
   * @return DecryptServer::DecryptRequest 組み立てたリクエスト
   */
  static DecryptServer::DecryptRequest makeServerTestRequest(const std::string& key,
                                                             const std::string& encryptedText)
  {
    DecryptServer::DecryptRequest request;
    request.keyLength = static_cast<uint8_t>(key.size());
    request.encryptedTextLength = static_cast<uint16_t>(encryptedText.size());
    request.iterations = 1;
    std::memcpy(request.key, key.data(), key.size());
    std::memcpy(request.encryptedText, encryptedText.data(), encryptedText.size());
    return request;
  }

  // 生成直後の状態が想定通りであることを確認する
  TEST(DecryptSocketServerTest, InitialState)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    int expected = -1;
    EXPECT_EQ(expected, server.getListenSocket());
    EXPECT_FALSE(server.getIsRunning());
    EXPECT_EQ(DecryptServer::DEFAULT_PORT, server.getPort());
  }

  // 初期化に成功した場合、稼働中になることを確認する
  TEST(DecryptSocketServerTest, InitSuccess)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    EXPECT_TRUE(server.init());
    EXPECT_TRUE(server.getIsRunning());
  }

  // socket()が失敗した場合、初期化に失敗することを確認する
  TEST(DecryptSocketServerTest, InitFailsOnSocketError)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.forceSocketError = true;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    EXPECT_FALSE(server.init());
    EXPECT_FALSE(server.getIsRunning());
  }

  // bind()が失敗した場合、初期化に失敗することを確認する
  TEST(DecryptSocketServerTest, InitFailsOnBindError)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.forceBindError = true;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    EXPECT_FALSE(server.init());
  }

  // listen()が失敗した場合、初期化に失敗することを確認する
  TEST(DecryptSocketServerTest, InitFailsOnListenError)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.forceListenError = true;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    EXPECT_FALSE(server.init());
  }

  // シャットダウン後は稼働中でなくなることを確認する
  TEST(DecryptSocketServerTest, ShutdownStopsServer)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    server.init();
    server.shutdown();

    int expected = -1;
    EXPECT_FALSE(server.getIsRunning());
    EXPECT_EQ(expected, server.getListenSocket());
  }

  // ポート番号を設定できることを確認する
  TEST(DecryptSocketServerTest, SetPort)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    int expected = 30000;
    server.setPort(expected);
    EXPECT_EQ(expected, server.getPort());
  }

  // 復号リクエストを受信すると、復号結果を返すことを確認する
  TEST(DecryptSocketServerTest, HandleDecryptCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    DecryptServer::DecryptRequest request
        = makeServerTestRequest("1234", SERVER_TEST_ENCRYPTED_TEXT);
    mockNet.pushRecvData(&request, sizeof(request));

    server.handleConnection(888);

    ASSERT_EQ(sizeof(DecryptServer::DecryptResponse), mockNet.sentBytes.size());
    DecryptServer::DecryptResponse response;
    std::memcpy(&response, mockNet.sentBytes.data(), sizeof(response));

    std::string expected = "Hello";
    EXPECT_TRUE(response.success);
    EXPECT_EQ(expected, std::string(response.plainText, response.plainTextLength));
  }

  // 受信が1バイトずつに分割されても、正しく復号できることを確認する
  TEST(DecryptSocketServerTest, HandleDecryptCommandWithPartialReceive)
  {
    MockByteStreamNetworkSystem mockNet;
    mockNet.maxRecvChunk = 1;  // 部分受信を強制的に再現する
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    DecryptServer::DecryptRequest request
        = makeServerTestRequest("1234", SERVER_TEST_ENCRYPTED_TEXT);
    mockNet.pushRecvData(&request, sizeof(request));

    server.handleConnection(888);

    ASSERT_EQ(sizeof(DecryptServer::DecryptResponse), mockNet.sentBytes.size());
    DecryptServer::DecryptResponse response;
    std::memcpy(&response, mockNet.sentBytes.data(), sizeof(response));

    std::string expected = "Hello";
    EXPECT_TRUE(response.success);
    EXPECT_EQ(expected, std::string(response.plainText, response.plainTextLength));
  }

  // 1接続の中で複数のリクエストを処理できることを確認する
  TEST(DecryptSocketServerTest, HandleMultipleRequestsInOneConnection)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    DecryptServer::DecryptRequest request
        = makeServerTestRequest("1234", SERVER_TEST_ENCRYPTED_TEXT);
    mockNet.pushRecvData(&request, sizeof(request));
    mockNet.pushRecvData(&request, sizeof(request));

    server.handleConnection(888);

    size_t expected = sizeof(DecryptServer::DecryptResponse) * 2;
    EXPECT_EQ(expected, mockNet.sentBytes.size());
  }

  // Pingリクエストを受信すると、Pingレスポンスを返すことを確認する
  TEST(DecryptSocketServerTest, HandlePingCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    DecryptServer::PingRequest request;
    mockNet.pushRecvData(&request, sizeof(request));

    server.handleConnection(888);

    size_t expected = sizeof(DecryptServer::PingResponse);
    EXPECT_EQ(expected, mockNet.sentBytes.size());
  }

  // 切断要求を受信すると、レスポンスを返さず処理を終えることを確認する
  TEST(DecryptSocketServerTest, HandleDisconnectCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);
    server.init();

    DecryptServer::Command command = DecryptServer::Command::DISCONNECT;
    mockNet.pushRecvData(&command, DecryptServer::COMMAND_SIZE);

    server.handleConnection(888);

    size_t expected = 0;
    EXPECT_EQ(expected, mockNet.sentBytes.size());
    EXPECT_TRUE(server.getIsRunning());  // 切断要求ではサーバーは停止しない
  }

  // シャットダウン要求を受信すると、サーバーが停止することを確認する
  TEST(DecryptSocketServerTest, HandleShutdownCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);
    server.init();

    DecryptServer::Command command = DecryptServer::Command::SHUTDOWN;
    mockNet.pushRecvData(&command, DecryptServer::COMMAND_SIZE);

    server.handleConnection(888);

    EXPECT_FALSE(server.getIsRunning());
  }

  // 未知のコマンドを受信しても、レスポンスを返さず安全に処理を終えることを確認する
  TEST(DecryptSocketServerTest, HandleUnknownCommand)
  {
    MockByteStreamNetworkSystem mockNet;
    DecryptActionHandler handler;
    DecryptSocketServer server(handler, mockNet);

    uint8_t unknownCommand = 100;
    mockNet.pushRecvData(&unknownCommand, sizeof(unknownCommand));

    server.handleConnection(888);

    size_t expected = 0;
    EXPECT_EQ(expected, mockNet.sentBytes.size());
  }

}  // namespace etrobocon2026_test
