/**
 * @file Logger.cpp
 * @brief 走行中のログを出力、保存するクラス
 * @author miyahara046
 */

#include "Logger.h"

using namespace std;

// ログのバッファとインデックスの初期化
char Logger::logs[LOG_BUFFER_SIZE] = "";
int Logger::currentIndex = 0;
string Logger::fileName = string(DEFAULT_LOG_FILE_PATH) + DEFAULT_LOG_FILE_NAME;

// ログの初期化
void Logger::init()
{
  logs[0] = '\0';
  currentIndex = 0;
}

// 標準ログの出力
void Logger::info(const char* message)
{
  write(INFO, message);
}

// 警告ログの出力
void Logger::warning(const char* message)
{
  write(WARNING, message);
}

// エラーログの出力
void Logger::error(const char* message)
{
  write(ERROR, message);
}

// デバッグログの出力
void Logger::debug(const char* message)
{
  write(DEBUG, message);
}

// 可変長に対応したログの出力
void Logger::printfLog(Level level, const char* format, ...)
{
  char buffer[MESSAGE_BUFFER_SIZE];

  // 可変引数の処理
  va_list args;

  va_start(args, format);

  vsnprintf(buffer, sizeof(buffer), format, args);

  va_end(args);

  write(level, buffer);
}

// 出力するログを整形して保存する
void Logger::write(Level level, const char* message)
{
  char output[MESSAGE_BUFFER_SIZE];
  int time = ClockUtil::now();

  // ログレベルとメッセージを整形する
  snprintf(output, sizeof(output), "[%d ms][%s] %s\n", time, levelToString(level), message);

  // ターミナルへの出力
  printf("%s", levelToColor(level));
  printf("%s", output);
  printf("\x1b[39m");

  // バッファサイズを超えないか確認
  int remainBuffer = LOG_BUFFER_SIZE - currentIndex - 1;

  if(remainBuffer <= 0) {
    return;
  }

  // ログを書き込む
  int written = snprintf(&logs[currentIndex], remainBuffer, "%s", output);

  // 書き込んだ文字数を更新
  if(written >= remainBuffer) {
    currentIndex = LOG_BUFFER_SIZE - 1;
  } else if(written > 0) {
    currentIndex += written;
  }
}

// ログファイルの出力先変更
void Logger::setFileName(const std::string& name, const std::string& path)
{
  std::string finalName = name.empty() ? DEFAULT_LOG_FILE_NAME : name;
  std::string finalPath = path.empty() ? DEFAULT_LOG_FILE_PATH : path;

  Logger::fileName = finalPath + finalName;
}

// ログファイルの出力
void Logger::outputToFile()
{
  std::filesystem::path outputPath(Logger::fileName);

  // 出力先のディレクトリが存在しない場合は作成する
  if(!outputPath.parent_path().empty()) {
    try {
      std::filesystem::create_directories(outputPath.parent_path());
    } catch(...) {
      warning("Logger:Failed to create log directory");
      return;
    }
  }

  // ログファイルをバイナリモードで開く
  std::ofstream file(outputPath, std::ios::binary | std::ios::app);

  if(!file.is_open()) {
    warning("Logger:Failed to open or create log file");
    return;
  }

  // ログをファイルに書き込む
  file.write(logs, currentIndex);

  file.flush();

  if(!file.good()) {
    warning("Logger:Failed to write log file");
  }

  init();
}

// ログレベルを文字列に変換
const char* Logger::levelToString(Level level)
{
  switch(level) {
    // 標準ログ
    case INFO:
      return "INFO";

    // 警告ログ
    case WARNING:
      return "WARNING";

    // エラーログ
    case ERROR:
      return "ERROR";

    // デバッグログ
    case DEBUG:
      return "DEBUG";

    // その他
    default:
      return "UNKNOWN";
  }
}

// ログレベルに応じた色コード
const char* Logger::levelToColor(Level level)
{
  switch(level) {
    case INFO:
      // 白色(櫻坂46)
      return "\x1b[37m";

    case WARNING:
      // 黄色(日向坂46:ハッピーオーラ)
      return "\x1b[33m";

    case ERROR:
      // 赤色(欅坂46:不協和音)
      return "\x1b[31m";

    case DEBUG:
      // 紫色(乃木坂46)
      return "\x1b[35m";

    // その他の色コード:
    // \x1b[30m(黒)
    // \x1b[31m(赤)
    // \x1b[32m(緑)
    // \x1b[33m(黄)
    // \x1b[34m(青)
    // \x1b[35m(マゼンタ)
    // \x1b[37m(白)
    default:
      return "\x1b[39m";
  }
}