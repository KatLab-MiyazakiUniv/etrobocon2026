/**
 * @file   main.cpp
 * @brief  復号サーバーのエントリポイント
 * @author HaruArima08
 */

#include <cstdlib>
#include <cstring>
#include "DecryptProtocol.h"
#include "DecryptActionHandler.h"
#include "DecryptSocketServer.h"
#include "RealNetworkSystem.h"
#include "Logger.h"

/**
 * @brief コマンドライン引数からポート番号を取得する
 * @param argc 引数の数
 * @param argv 引数の配列
 * @return ポート番号（指定が無ければデフォルト値）
 */
static int parsePort(int argc, char* argv[])
{
  for(int i = 1; i < argc - 1; i++) {
    if(std::strcmp(argv[i], "--port") == 0) {
      return std::atoi(argv[i + 1]);
    }
  }
  return DecryptServer::DEFAULT_PORT;
}

int main(int argc, char* argv[])
{
  Logger::info("復号サーバーを起動します");

  RealNetworkSystem realNetworkSystem;
  DecryptActionHandler decryptHandler;
  DecryptSocketServer server(decryptHandler, realNetworkSystem, parsePort(argc, argv));

  if(!server.init()) {
    Logger::error("復号サーバーの初期化に失敗しました");
    return -1;
  }

  // 接続を待ち受け続ける（SHUTDOWNコマンドを受信するまで戻らない）
  server.run();

  Logger::info("復号サーバーを終了します");
  return 0;
}
