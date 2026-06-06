/**
 * @file SocketServerTest.cpp
 * @brief SocketServerクラスをテストする
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include "SocketServer.h"
#include "MockNetworkSystem.h"

// インスタンスに指定したportを代入できているかを確認
TEST(SocketServerTest, ConstructorSetsPortCorrectly)
{
  MockNetworkSystem mockNet;
  int testPort = 12345;
  SocketServer server(mockNet, testPort);
  EXPECT_EQ(testPort, server.port);
}

// CameraServer::realを参照しているかアドレスの一致で確認
TEST(SocketServerTest, CompareAddress)
{
  SocketServer server(CameraServer::real);
  SocketServer server2(CameraServer::real);
  EXPECT_EQ(&CameraServer::real, &server.netSys);
  EXPECT_EQ(&server2.netSys, &server.netSys);
  EXPECT_EQ(&CameraServer::real, &server2.netSys);
}

// メンバ変数のlistenSocketの初期値が-1か
TEST(SocketServerTest, CheckDefaultListenSocket)
{
  MockNetworkSystem mockNet;
  SocketServer server(mockNet);
  int expectedDefaultListenSocket = -1;
  EXPECT_EQ(server.listenSocket, expectedDefaultListenSocket);
}

// メンバ変数のisRunningの初期値がfalse
TEST(SocketServerTest, DefaultisRunning)
{
  MockNetworkSystem mockNet;
  SocketServer server(mockNet);
  EXPECT_FALSE(server.isRunning);
}

// メンバ変数のportのデフォルト引数がDEFAULT_PORTかを確認
TEST(SocketServerTest, CheckdefaultVariable2)
{
  MockNetworkSystem mockNet;
  SocketServer server(mockNet);
  EXPECT_EQ(server.port, CameraServer::DEFAULT_PORT);
}

// shutdown()の実行後にisRunningとlistenSocketがデフォルト値に戻っているか
TEST(SocketServerTest, ShutdownChangesStateCorrectly)
{
  MockNetworkSystem mockNet;
  SocketServer server(mockNet);
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

// クライアントからSHUTDOWNコマンドを受信した際の、サーバーのシャットダウン挙動テスト
TEST(SocketServerTest, HandleConnectionShutdown)
{
  MockNetworkSystem mockNet;
  mockNet.receiveCommand = true;
  mockNet.recvBuff = reinterpret_cast<void*>(CameraServer::Command::SHUTDOWN);
  SocketServer server(mockNet);
  server.isRunning = true;
  server.listenSocket = 100;
  int afterConnectListenSocket = -1;
  int dummyClientSocket = 200;
  server.handleConnection(dummyClientSocket);
  EXPECT_FALSE(server.isRunning);
  EXPECT_EQ(server.listenSocket, afterConnectListenSocket);
}
