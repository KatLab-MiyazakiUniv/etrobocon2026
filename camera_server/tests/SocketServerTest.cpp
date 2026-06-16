/**
 * @file SocketServerTest.cpp
 * @brief SocketServerクラスをテストする
 * @author sadomiya-sousi, takuchi17
 */

#include <gtest/gtest.h>
#include "SocketServer.h"
#include "MockNetworkSystem.h"
#include "RealNetworkSystem.h"

namespace etrobocon2026_test {
  // インスタンスに指定したportを代入できているかを確認
  TEST(SocketServerTest, ConstructorSetsPortCorrectly)
  {
    MockNetworkSystem mockNet;
    int testPort = 12345;
    SocketServer server(mockNet, testPort);
    EXPECT_EQ(testPort, server.getPort());
  }

  // メンバ変数のlistenSocketの初期値が-1か
  TEST(SocketServerTest, CheckDefaultListenSocket)
  {
    MockNetworkSystem mockNet;
    SocketServer server(mockNet);
    int expectedDefaultListenSocket = -1;
    EXPECT_EQ(server.getListenSocket(), expectedDefaultListenSocket);
  }

  // メンバ変数のisRunningの初期値がfalse
  TEST(SocketServerTest, DefaultisRunning)
  {
    MockNetworkSystem mockNet;
    SocketServer server(mockNet);
    EXPECT_FALSE(server.getIsRunning());
  }

  // メンバ変数のportのデフォルト引数がDEFAULT_PORTかを確認
  TEST(SocketServerTest, CheckdefaultVariable2)
  {
    MockNetworkSystem mockNet;
    SocketServer server(mockNet);
    EXPECT_EQ(server.getPort(), CameraServer::DEFAULT_PORT);
  }

  // shutdown()の実行後にisRunningとlistenSocketがデフォルト値に戻っているか
  TEST(SocketServerTest, ShutdownChangesStateCorrectly)
  {
    MockNetworkSystem mockNet;
    SocketServer server(mockNet);
    server.init();
    server.shutdown();
    EXPECT_FALSE(server.getIsRunning());
    EXPECT_EQ(server.getListenSocket(), -1);
  }

  // socket()に失敗した時に分岐先でfalseを返す事を確認する
  TEST(SocketServerTest, InitFailsOnSocketError)
  {
    MockNetworkSystem mockNet;
    mockNet.forceSocketError = true;
    SocketServer server(mockNet);
    EXPECT_FALSE(server.init());
  }

  // bind()に失敗した時に分岐先でfalseを返す事を確認する
  TEST(SocketServerTest, InitFailsOnBindError)
  {
    MockNetworkSystem mockNet;
    mockNet.forceBindError = true;
    SocketServer server(mockNet);
    EXPECT_FALSE(server.init());
  }

  // serverの初期化に成功する事を確認する
  TEST(SocketServerTest, InitSuccessTest)
  {
    MockNetworkSystem mockNet;
    SocketServer server(mockNet);
    EXPECT_TRUE(server.init());
  }

  // クライアントからSHUTDOWNコマンドを受信した際にソケットを閉じることを確認する
  TEST(SocketServerTest, HandleConnectionShutdown)
  {
    MockNetworkSystem mockNet;
    mockNet.hasRecvData = true;
    mockNet.recvData = CameraServer::Command::SHUTDOWN;
    SocketServer server(mockNet);
    server.init();
    server.run();
    EXPECT_FALSE(server.getIsRunning());
    EXPECT_EQ(server.getListenSocket(), -1);
  }

  // このテストはクライアントからDISCONNECTコマンドを受信した際に接続待ちに戻ることを確認するテストであるが、現状の実装ではrun()が無限ループに入ってしまうため、DISCONNECTコマンドを受信した際に接続待ちに戻ることを確認するテストは現状行えない。
  // // クライアントからDISCONNECTコマンドを受信した際に接続待ちに戻ることを確認する
  // TEST(SocketServerTest, HandleConnectionDisconnect)
  // {
  //   MockNetworkSystem mockNet;
  //   mockNet.hasRecvData = true;
  //   mockNet.recvData = CameraServer::Command::DISCONNECT;
  //   SocketServer server(mockNet);
  //   server.init();
  //   server.run();
  //   EXPECT_TRUE(server.getIsRunning());
  //   EXPECT_EQ(server.getListenSocket(), 999);  // モックのsocket()が999を返すため
  // }
}  // namespace etrobocon2026_test
