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
std::string CsvLogger::fileName
    = std::string(DEFAULT_CSV_LOG_FILE_PATH) + DEFAULT_CSV_LOG_FILE_NAME;

// CSVヘッダーの定義
const std::vector<std::string> CsvLogger::HEADERS
    = { "time", "brightness", "rightPower", "leftPower" };

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

  // 配列からカンマ区切りのヘッダー文字列を構築
  std::string headerStr = "";
  for(size_t i = 0; i < HEADERS.size(); ++i) {
    headerStr += HEADERS[i];
    if(i < HEADERS.size() - 1) {
      headerStr += ",";
    }
  }
  headerStr += "\n";

  int written = snprintf(&logs[currentIndex], remainBuffer, "%s", headerStr.c_str());

  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// 各種値を追加する
void CsvLogger::add(const LogData& data)
{
  int remainBuffer = LOG_BUFFER_SIZE - currentIndex - 1;
  if(remainBuffer <= 0) {
    return;
  }

  // optionalの値を判定して、あれば数値、なければ空文字にするラムダ関数
  auto appendField = [](std::string& row, const std::optional<int>& field, bool isLast) {
    if(field.has_value()) {
      row += std::to_string(field.value());
    }
    if(!isLast) {
      row += ",";
    }
  };

  // 一行分のCSVデータを構築
  std::string rowStr = "";
  appendField(rowStr, data.time, false);
  appendField(rowStr, data.brightness, false);
  appendField(rowStr, data.rightPower, false);
  appendField(rowStr, data.leftPower, true);
  rowStr += "\n";

  int written = snprintf(&logs[currentIndex], remainBuffer, "%s", rowStr.c_str());

  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// ログファイルの出力先変更
void CsvLogger::setFileName(const std::string& name, const std::string& path)
{
  std::string finalName = name.empty() ? DEFAULT_CSV_LOG_FILE_NAME : name;
  std::string finalPath = path.empty() ? DEFAULT_CSV_LOG_FILE_PATH : path;

  CsvLogger::fileName = finalPath + finalName;
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

  // ファイルが既に存在するかチェック
  bool isNewFile
      = !std::filesystem::exists(outputPath) || std::filesystem::file_size(outputPath) == 0;

  // CSVファイルをバイナリモードで開く
  std::ofstream file(outputPath, std::ios::binary | std::ios::app);

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

  // ファイルに書き出し終えたら、メモリ上のバッファをクリアして次の走行ログに備える
  init();
}