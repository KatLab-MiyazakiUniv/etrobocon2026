/**
 * @file CsvLogger.cpp
 * @brief 走行中の各種値可視化のためのcsvファイル作成に関するクラス
 * @author miyahara046
 */

#include "CsvLogger.h"

// CSVログのバッファとインデックスの初期化
char CsvLogger::logs[LOG_BUFFER_SIZE] = "";
int CsvLogger::currentIndex = 0;
// 現在のログファイルパス
std::string CsvLogger::fileName = "etrobocon2026/datafiles/logfiles/runlog.csv";

// CSVログの初期化
void CsvLogger::init()
{
  logs[0] = '\0';
  currentIndex = 0;
}

// CSVヘッダを書き込む
void CsvLogger::writeHeader()
{
  int remainBuffer = LOG_BUFFER_SIZE - currentIndex - 1;

  if(remainBuffer <= 0) {
    return;
  }

  int written
      = snprintf(&logs[currentIndex], remainBuffer, "time,brightness,rightPower,leftPower\n");

  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// 各種値を追加する
void CsvLogger::add(int brightness, int rightPower, int leftPower)
{
  int remainBuffer = LOG_BUFFER_SIZE - currentIndex - 1;

  if(remainBuffer <= 0) {
    return;
  }

  // int time = ClockUtil::now();

  int written = snprintf(&logs[currentIndex], remainBuffer, "%d,%d,%d\n",
                         //  "%d,%d,%d,%d\n",
                         //  time,
                         brightness, rightPower, leftPower);

  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// ログファイルの出力先変更
void CsvLogger::setFileName(const std::string& path)
{
  if(!path.empty()) {
    CsvLogger::fileName = path;
  }
}

// ログファイルの出力
void CsvLogger::outputToFile()
{
  std::filesystem::path outputPath(CsvLogger::fileName);

  // 出力先のディレクトリが存在しない場合は作成する
  if(!outputPath.parent_path().empty()) {
    try {
      std::filesystem::create_directories(outputPath.parent_path());
    } catch(...) {
      std::cerr << "CsvLogger:failed to create csv directory" << std::endl;
      return;
    }
  }

  // CSVファイルをバイナリモードで開く
  std::ofstream file(outputPath, std::ios::binary | std::ios::trunc);

  if(!file.is_open()) {
    std::cerr << "CsvLogger: failed to open or create csv file" << std::endl;
    return;
  }

  // ログをCSVファイルに書き込む
  file.write(logs, currentIndex);
  file.flush();
  if(!file.good()) {
    std::cerr << "CsvLogger: failed to write csv file" << std::endl;
  }
}