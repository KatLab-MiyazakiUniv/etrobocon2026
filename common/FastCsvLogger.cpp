/**
 * @file FastCsvLogger.cpp
 * @brief 走行中の実測値をcsvファイルに出力するためのクラス（高速版）
 */

#include "FastCsvLogger.h"

// 静的メンバ変数の初期化
std::vector<LogData> FastCsvLogger::logBuffer;
bool FastCsvLogger::hasHeader = false;
std::string FastCsvLogger::fileName
    = std::string(DEFAULT_FAST_CSV_LOG_FILE_PATH) + DEFAULT_FAST_CSV_LOG_FILE_NAME;

const std::vector<std::string> FastCsvLogger::HEADERS
    = { "time",      "comand:id",  "brightness", "rightPower", "leftPower", "rightSpeed",
        "leftSpeed", "currentVal", "target",     "kp",         "ki",        "kd" };

// CSVログの初期化
void FastCsvLogger::init()
{
  logBuffer.clear();
  logBuffer.reserve(MAX_LOG_SIZE);
  hasHeader = false;
}

// CSVヘッダを出力対象に設定
void FastCsvLogger::writeHeader()
{
  hasHeader = true;
}

// 各種値を追加する（メモリへのコピーのみで高速）
void FastCsvLogger::add(const LogData& data)
{
  if(logBuffer.size() >= MAX_LOG_SIZE) {
    return;
  }

  LogData localData = data;

  if(!localData.time.has_value()) {
    localData.time = ClockUtil::now();
  }

  logBuffer.push_back(localData);
}

// ログファイルの出力先変更
void FastCsvLogger::setFileName(const std::string& name, const std::string& path)
{
  std::string finalName = name.empty() ? DEFAULT_FAST_CSV_LOG_FILE_NAME : name;
  std::string finalPath = path.empty() ? DEFAULT_FAST_CSV_LOG_FILE_PATH : path;

  FastCsvLogger::fileName = finalPath + finalName;
}

// ログファイルの出力
void FastCsvLogger::outputToFile()
{
  std::filesystem::path outputPath(FastCsvLogger::fileName);

  // 出力先のディレクトリが存在しない場合は作成する
  if(!outputPath.parent_path().empty()) {
    try {
      std::filesystem::create_directories(outputPath.parent_path());
    } catch(...) {
      Logger::error("FastCsvLogger:failed to create csv directory");
      return;
    }
  }

  // CSVファイルをバイナリモードで開く
  std::ofstream file(outputPath, std::ios::binary | std::ios::trunc);

  if(!file.is_open()) {
    Logger::error("FastCsvLogger: failed to open or create csv file");
    return;
  }

  // ヘッダーを出力
  if(hasHeader) {
    std::string headerStr = "";
    for(size_t i = 0; i < HEADERS.size(); ++i) {
      headerStr += HEADERS[i];
      if(i < HEADERS.size() - 1) {
        headerStr += ",";
      }
    }
    headerStr += "\n";
    file.write(headerStr.c_str(), headerStr.size());
  }

  // optionalの値を判定して、あれば数値、なければ空文字にするラムダ関数
  auto appendField = [](std::string& row, const auto& field, bool isLast) {
    if(field.has_value()) {
      using FieldType = std::decay_t<decltype(field.value())>;
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

  // ログデータをCSVに書き出す
  for(const auto& logData : logBuffer) {
    std::string rowStr = "";
    appendField(rowStr, logData.time, false);
    appendField(rowStr, logData.id, false);
    appendField(rowStr, logData.brightness, false);
    appendField(rowStr, logData.rightPower, false);
    appendField(rowStr, logData.leftPower, false);
    appendField(rowStr, logData.rightSpeed, false);
    appendField(rowStr, logData.leftSpeed, false);
    appendField(rowStr, logData.currentVal, false);
    appendField(rowStr, logData.target, false);
    appendField(rowStr, logData.kp, false);
    appendField(rowStr, logData.ki, false);
    appendField(rowStr, logData.kd, true);
    rowStr += "\n";

    file.write(rowStr.c_str(), rowStr.size());
  }

  file.flush();
  if(!file.good()) {
    Logger::error("FastCsvLogger: failed to write csv file");
  }

  // ファイルに書き出し終えたら、メモリ上のバッファをクリアして次の走行ログに備える
  init();
}
