/**
 * @file CsvLogger.cpp
 * @brief 走行中の実測値をcsvファイルに出力するためのクラス
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
    = { "time",      "comand:id",  "brightness", "rightPower", "leftPower", "rightSpeed",
        "leftSpeed", "currentVal", "target",     "kp",         "ki",        "kd" };

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
  LogData localData = data;

  if(!localData.time.has_value()) {
    localData.time = ClockUtil::now();
  }

  int remainBuffer = LOG_BUFFER_SIZE - currentIndex - 1;
  if(remainBuffer <= 0) {
    return;
  }

  // optionalの値を判定して、あれば数値、なければ空文字にするラムダ関数
  auto appendField = [](std::string& row, const auto& field, bool isLast) {
    if(field.has_value()) {
      using FieldType = std::decay_t<decltype(field.value())>;
      // 元々文字列のものはそのままで、数値の場合は文字列化する
      if constexpr(std::is_same_v<FieldType, std::string>) {
        row += field.value();
      } else {
        std::string valStr = std::to_string(field.value());
        if(valStr.find('.') != std::string::npos) {
          valStr.erase(valStr.find_last_not_of('0') + 1, std::string::npos);
          if(valStr.back() == '.') {
            valStr.pop_back();
          }
        }
        row += valStr;
      }
    }
    if(!isLast) {
      row += ",";
    }
  };
  // 一行分のCSVデータを構築
  std::string rowStr = "";
  appendField(rowStr, localData.time, false);
  appendField(rowStr, localData.id, false);
  appendField(rowStr, localData.brightness, false);
  appendField(rowStr, localData.rightPower, false);
  appendField(rowStr, localData.leftPower, false);
  appendField(rowStr, localData.rightSpeed, false);
  appendField(rowStr, localData.leftSpeed, false);
  appendField(rowStr, localData.currentVal, false);
  appendField(rowStr, localData.target, false);
  appendField(rowStr, localData.kp, false);
  appendField(rowStr, localData.ki, false);
  appendField(rowStr, localData.kd, true);
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
      Logger::error("CsvLogger:failed to create csv directory");
      return;
    }
  }

  // CSVファイルをバイナリモードで開く
  std::ofstream file(outputPath, std::ios::binary | std::ios::trunc);

  if(!file.is_open()) {
    Logger::error("CsvLogger: failed to open or create csv file");
    return;
  }

  // ログをCSVファイルに書き込む
  file.write(logs, currentIndex);
  file.flush();
  if(!file.good()) {
    Logger::error("CsvLogger: failed to write csv file");
  }

  // ファイルに書き出し終えたら、メモリ上のバッファをクリアして次の走行ログに備える
  init();
}