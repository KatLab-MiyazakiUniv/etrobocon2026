/**
 * @file   bench_main.cpp
 * @brief  復号処理の配置（ラズパイ側／PC側）による性能差を計測するエントリポイント
 * @author HaruArima08
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "BenchmarkRunner.h"
#include "BenchmarkCsvWriter.h"
#include "DecryptClient.h"
#include "DecryptMode.h"
#include "DecryptProtocol.h"
#include "LocalDecryptService.h"
#include "RemoteDecryptService.h"
#include "RealNetworkSystem.h"
#include "Logger.h"

namespace {

  // 計測に用いる既定の暗号文と復号キー（DecrypterTestと同じ組み合わせ）
  constexpr const char* DEFAULT_KEY = "1234";
  constexpr const char* DEFAULT_ENCRYPTED_TEXT = "U2FsdGVkX1+kU2ekK92ZjOQTL/ytu4bgR0tAJllMG6I=";
  constexpr const char* DEFAULT_PLAIN_TEXT = "Hello";
  constexpr const char* DEFAULT_CSV_FILE_PATH = "datafiles/logfiles/decrypt_benchmark.csv";

  /**
   * @brief 使い方を標準出力へ表示する
   */
  void printUsage()
  {
    printf("使い方: ./decrypt_bench [オプション]\n"
           "  --trials <回数>      計測する試行回数（既定値: 100）\n"
           "  --warmup <回数>      計測前に捨てる試行回数（既定値: 10）\n"
           "  --iterations <回数>  1リクエストあたりの復号回数（既定値: 1）\n"
           "  --remote-ip <IP>     PC側復号サーバーのIPアドレス（既定値: 計測をスキップ）\n"
           "  --port <番号>        復号サーバーのポート番号（既定値: %d）\n"
           "  --key <文字列>       復号キー（既定値: %s）\n"
           "  --text <文字列>      暗号文（既定値: 組み込みのテスト用暗号文）\n"
           "  --expect <文字列>    期待する平文（既定値: %s）\n"
           "  --out <パス>         CSVの出力先（既定値: %s）\n",
           DecryptServer::DEFAULT_PORT, DEFAULT_KEY, DEFAULT_PLAIN_TEXT, DEFAULT_CSV_FILE_PATH);
  }

  /**
   * @brief コマンドライン引数から指定されたオプションの値を取得する
   * @param argc 引数の数
   * @param argv 引数の配列
   * @param name 探すオプション名
   * @param defaultValue 見つからなかった場合の既定値
   * @return オプションの値
   */
  std::string getOption(int argc, char* argv[], const char* name, const std::string& defaultValue)
  {
    for(int i = 1; i < argc - 1; i++) {
      if(std::strcmp(argv[i], name) == 0) return std::string(argv[i + 1]);
    }
    return defaultValue;
  }

}  // namespace

int main(int argc, char* argv[])
{
  if(argc > 1 && std::strcmp(argv[1], "--help") == 0) {
    printUsage();
    return 0;
  }

  BenchmarkCondition condition;
  condition.trials = std::atoi(getOption(argc, argv, "--trials", "100").c_str());
  condition.warmupTrials = std::atoi(getOption(argc, argv, "--warmup", "10").c_str());
  condition.iterations
      = static_cast<uint32_t>(std::atoi(getOption(argc, argv, "--iterations", "1").c_str()));
  condition.key = getOption(argc, argv, "--key", DEFAULT_KEY);
  condition.encryptedText = getOption(argc, argv, "--text", DEFAULT_ENCRYPTED_TEXT);
  condition.expectedPlainText = getOption(argc, argv, "--expect", DEFAULT_PLAIN_TEXT);
  condition.csvFilePath = getOption(argc, argv, "--out", DEFAULT_CSV_FILE_PATH);
  condition.remoteServerIp = getOption(argc, argv, "--remote-ip", "");
  condition.port = std::atoi(
      getOption(argc, argv, "--port", std::to_string(DecryptServer::DEFAULT_PORT)).c_str());

  if(condition.trials <= 0) {
    Logger::error("試行回数は1以上を指定してください");
    return -1;
  }

  BenchmarkCsvWriter csvWriter(condition.csvFilePath);
  if(!csvWriter.open()) return -1;

  BenchmarkRunner runner(condition, csvWriter);
  std::vector<BenchmarkModeResult> results;

  // LOCAL_INPROC: 通信を挟まない下限値
  LocalDecryptService localService;
  results.push_back(runner.run(localService, "-"));

  // LOCAL_SOCKET: ラズパイ上の復号サーバーへローカル接続（案B）
  RealNetworkSystem localNetworkSystem;
  DecryptClient localClient(localNetworkSystem, condition.port, DecryptServer::LOCAL_SERVER_IP);
  if(localClient.connectToServer()) {
    RemoteDecryptService localSocketService(localClient, DecryptMode::LOCAL_SOCKET);
    results.push_back(runner.run(localSocketService, DecryptServer::LOCAL_SERVER_IP));
    localClient.disconnectFromServer();
  } else {
    Logger::error("ローカルの復号サーバーへ接続できないため、LOCAL_SOCKETの計測をスキップします");
  }

  // REMOTE: PC上の復号サーバーへ無線接続（案A）
  if(condition.remoteServerIp.empty()) {
    Logger::info("--remote-ipが未指定のため、REMOTEの計測をスキップします");
  } else {
    RealNetworkSystem remoteNetworkSystem;
    DecryptClient remoteClient(remoteNetworkSystem, condition.port,
                               condition.remoteServerIp.c_str());
    if(remoteClient.connectToServer()) {
      RemoteDecryptService remoteService(remoteClient, DecryptMode::REMOTE);
      results.push_back(runner.run(remoteService, condition.remoteServerIp));
      remoteClient.disconnectFromServer();
    } else {
      Logger::error("PC側の復号サーバーへ接続できないため、REMOTEの計測をスキップします");
    }
  }

  csvWriter.close();
  BenchmarkRunner::printSummary(results, condition);
  printf("計測結果を %s に出力しました\n", condition.csvFilePath.c_str());
  return 0;
}
