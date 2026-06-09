/**
 * @file SocketServerTest.cpp
 * @brief SocketServerクラスをテストする
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include "SocketServer.h"
#include "MockNetworkSystem.h"
#include "RealNetworkSystem.h"

namespace etrobocon2026_test {
  // インスタンスに指定したportを代入できているかを確認

  CameraCapture camera;
  ColorRegionDetectionActionHandler colorRegionDetectionHandler(camera);

  TEST(SocketServerTest, ConstructorSetsPortCorrectly)
  {
    MockNetworkSystem mockNet;
    int testPort = 12345;
    SocketServer server(colorRegionDetectionHandler, mockNet, testPort);
    EXPECT_EQ(testPort, server.port);
  }

  // SocketServerが注入されたNetworkSystemを参照しているかアドレスの一致で確認
  TEST(SocketServerTest, CompareAddress)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_EQ(&mockNet, &server.netSys);
    RealNetworkSystem realNet;
    SocketServer server2(colorRegionDetectionHandler, realNet);
    EXPECT_EQ(&realNet, &server2.netSys);
  }

  // メンバ変数のlistenSocketの初期値が-1か
  TEST(SocketServerTest, CheckDefaultListenSocket)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    int expectedDefaultListenSocket = -1;
    EXPECT_EQ(server.listenSocket, expectedDefaultListenSocket);
  }

  // メンバ変数のisRunningの初期値がfalse
  TEST(SocketServerTest, DefaultisRunning)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_FALSE(server.isRunning);
  }

  // メンバ変数のportのデフォルト引数がDEFAULT_PORTかを確認
  TEST(SocketServerTest, CheckdefaultVariable2)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_EQ(server.port, CameraServer::DEFAULT_PORT);
  }

  // shutdown()の実行後にisRunningとlistenSocketがデフォルト値に戻っているか
  TEST(SocketServerTest, ShutdownChangesStateCorrectly)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    server.isRunning = true;
    server.listenSocket = 100;
    server.shutdown();
    EXPECT_FALSE(server.isRunning);
    EXPECT_EQ(server.listenSocket, -1);
  }

  // socket()に失敗した時に分岐先でfalseを返す事を確認する
  TEST(SocketServerTest, InitFailsOnSocketError)
  {
    MockNetworkSystem mockNet;
    mockNet.forceSocketError = true;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_FALSE(server.init());
  }

  // bind()に失敗した時に分岐先でfalseを返す事を確認する
  TEST(SocketServerTest, InitFailsOnBindError)
  {
    MockNetworkSystem mockNet;
    mockNet.forceBindError = true;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_FALSE(server.init());
  }

  // serverの初期化に成功する事を確認する
  TEST(SocketServerTest, InitSuccessTest)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_TRUE(server.init());
  }

  // クライアントからSHUTDOWNコマンドを受信した際の、サーバーのシャットダウン挙動テスト
  TEST(SocketServerTest, HandleConnectionShutdown)
  {
    MockNetworkSystem mockNet;
    mockNet.receiveCommand = true;
    mockNet.recvBuff = reinterpret_cast<void*>(CameraServer::Command::SHUTDOWN);
    SocketServer server(colorRegionDetectionHandler, mockNet);
    server.isRunning = true;
    server.listenSocket = 100;
    int afterConnectListenSocket = -1;
    int dummyClientSocket = 200;
    server.handleConnection(dummyClientSocket);
    EXPECT_FALSE(server.isRunning);
    EXPECT_EQ(server.listenSocket, afterConnectListenSocket);
  }
}  // namespace etrobocon2026_test
