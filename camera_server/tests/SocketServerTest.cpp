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

  CameraCapture camera;
  ColorRegionDetectionActionHandler colorRegionDetectionHandler(camera);

  // インスタンスに指定したportを代入できているかを確認
  TEST(SocketServerTest, ConstructorSetsPortCorrectly)
  {
    MockNetworkSystem mockNet;
    int testPort = 12345;
    SocketServer server(colorRegionDetectionHandler, mockNet, testPort);
    EXPECT_EQ(testPort, server.getPort());
  }

  // メンバ変数のlistenSocketの初期値が-1か
  TEST(SocketServerTest, CheckDefaultListenSocket)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    int expectedDefaultListenSocket = -1;
    EXPECT_EQ(server.getListenSocket(), expectedDefaultListenSocket);
  }

  // メンバ変数のisRunningの初期値がfalse
  TEST(SocketServerTest, DefaultisRunning)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_FALSE(server.getIsRunning());
  }

  // メンバ変数のportのデフォルト引数がDEFAULT_PORTかを確認
  TEST(SocketServerTest, CheckdefaultVariable2)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    EXPECT_EQ(server.getPort(), CameraServer::DEFAULT_PORT);
  }

  // shutdown()の実行後にisRunningとlistenSocketがデフォルト値に戻っているか
  TEST(SocketServerTest, ShutdownChangesStateCorrectly)
  {
    MockNetworkSystem mockNet;
    SocketServer server(colorRegionDetectionHandler, mockNet);
    server.setIsRunning(true);
    server.setListenSocket(100);
    server.shutdown();
    EXPECT_FALSE(server.getIsRunning());
    EXPECT_EQ(server.getListenSocket(), -1);
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

  // クライアントからSHUTDOWNコマンドを受信した際にソケットを閉じることを確認する
  TEST(SocketServerTest, HandleConnectionShutdown)
  {
    MockNetworkSystem mockNet;
    mockNet.hasRecvData = true;
    mockNet.recvData = CameraServer::Command::SHUTDOWN;

    SocketServer server(colorRegionDetectionHandler, mockNet);
    server.setIsRunning(true);
    server.setListenSocket(100);
    int afterConnectListenSocket = -1;
    int dummyClientSocket = 200;
    server.handleConnection(dummyClientSocket);
    EXPECT_FALSE(server.getIsRunning());
    EXPECT_EQ(server.getListenSocket(), afterConnectListenSocket);
  }
}  // namespace etrobocon2026_test