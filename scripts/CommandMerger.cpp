/**
 * @file   CommandMerger.cpp
 * @brief  エリアコマンドのCSV結合と型チェックを行うスタンドアロンプログラム
 * @author nishijima515
 */

#include <array>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "../modules/Area.h"

constexpr char SEPARATOR = ',';

// etrobocon2026/ ルートからの相対パス
static const std::string AREA_PATH    = "datafiles/commands/Area/";
static const std::string MOTIONS_PATH = "datafiles/commands/Motions/";
static const std::string RUN_PATH     = "datafiles/commands/Run/";

// Area enum のインデックスに対応するエリア名（Area.h の定義順と一致させること）
static const std::array<std::string, 2> AREA_NAMES = { "LineTrace", "Area2" };

// 引数文字列 → Area enum の変換テーブル
static const std::unordered_map<std::string, Area> AREA_MAP = {
  { "LineTrace", LineTrace },
  { "Area2", Area2 },
};

static void trim(std::string& s)
{
  size_t start = s.find_first_not_of(" \t");
  if(start == std::string::npos) {
    s.clear();
    return;
  }
  size_t end = s.find_last_not_of(" \t");
  s = s.substr(start, end - start + 1);
}

// int型用：strtolで判定（v850クロスコンパイラでstd::stoiが使えない環境向けの実装と同様）
template <typename T>
bool tryParse(const std::string& s);

template <>
bool tryParse<int>(const std::string& s)
{
  const char* p = s.c_str();
  char* end;
  errno = 0;
  long x = std::strtol(p, &end, 10);
  if(p == end || *end != '\0') return false;
  if(errno == ERANGE || x < INT_MIN || x > INT_MAX) return false;
  return true;
}

// double型用：strtodで判定
template <>
bool tryParse<double>(const std::string& s)
{
  const char* p = s.c_str();
  char* end;
  errno = 0;
  std::strtod(p, &end);
  if(p == end || *end != '\0') return false;
  if(errno == ERANGE) return false;
  return true;
}

void createRunCSV(Area area, bool isLeftCourse)
{
  std::cout << "実行用コマンドファイルの作成を開始します" << std::endl;

  std::string areaName = AREA_NAMES[static_cast<int>(area)];
  std::string course   = isLeftCourse ? "Left" : "Right";

  std::string commandAreaFilePath = AREA_PATH + areaName + course + ".csv";
  std::string commandRunFilePath  = RUN_PATH + "run_" + areaName + course + ".csv";

  std::ifstream commandAreaFile(commandAreaFilePath);
  if(!commandAreaFile) {
    std::cerr << "エリアコマンドファイルを開けませんでした: " << commandAreaFilePath << std::endl;
    return;
  }

  std::ofstream commandRunFile(commandRunFilePath, std::ios::out | std::ios::trunc);
  if(!commandRunFile) {
    std::cerr << "実行用コマンドファイルを開けませんでした: " << commandRunFilePath << std::endl;
    return;
  }

  bool firstWrite = true;

  // 1行目（カラム名）をスキップ
  std::string headerLine;
  std::getline(commandAreaFile, headerLine);

  std::string areaFileLine;

  while(std::getline(commandAreaFile, areaFileLine)) {
    // コメント削除
    size_t commentPos = areaFileLine.find('#');
    if(commentPos != std::string::npos) {
      areaFileLine = areaFileLine.substr(0, commentPos);
    }
    trim(areaFileLine);
    if(areaFileLine.empty()) continue;

    // CSV分割
    std::stringstream ssArea(areaFileLine);
    std::vector<std::string> areaFileParams;
    std::string token;
    while(std::getline(ssArea, token, SEPARATOR)) {
      trim(token);
      areaFileParams.push_back(token);
    }

    if(areaFileParams.size() != 2) {
      std::cerr << "エリアコマンドファイルのフォーマットが不正です: " << areaFileLine << std::endl;
      continue;
    }

    std::string commandName = areaFileParams[0];
    std::string commandID   = areaFileParams[1];

    std::string commandMotionsFilePath = MOTIONS_PATH + commandName + ".csv";
    std::ifstream commandMotionsFile(commandMotionsFilePath);
    if(!commandMotionsFile) {
      std::cerr << "動作コマンドファイルを開けませんでした: " << commandMotionsFilePath << std::endl;
      continue;
    }

    // 1行目（カラム名）をスキップ
    std::string motionsHeaderLine;
    std::getline(commandMotionsFile, motionsHeaderLine);

    bool found = false;
    std::string motionsFileLine;
    while(std::getline(commandMotionsFile, motionsFileLine)) {
      // コメント削除
      size_t motionsCommentPos = motionsFileLine.find('#');
      if(motionsCommentPos != std::string::npos) {
        motionsFileLine = motionsFileLine.substr(0, motionsCommentPos);
      }
      trim(motionsFileLine);
      if(motionsFileLine.empty()) continue;

      std::stringstream ssMotions(motionsFileLine);
      std::vector<std::string> motionParams;
      while(std::getline(ssMotions, token, SEPARATOR)) {
        trim(token);
        motionParams.push_back(token);
      }

      // ID一致チェック（2列目がID想定）
      if(motionParams.size() >= 2 && motionParams[1] == commandID) {
        if(!firstWrite) commandRunFile << "\n";
        commandRunFile << motionsFileLine;
        firstWrite = false;
        found      = true;
        break;
      }
    }

    if(!found) {
      std::cerr << "コマンド " << commandName << " に ID=" << commandID << " が見つかりませんでした"
                << std::endl;
    }
  }

  commandRunFile.close();
  std::cout << "実行用コマンドファイルの作成が完了しました: " << commandRunFilePath << std::endl;
}

bool checkType(const std::string& commandFilePath)
{
  std::cout << "型チェックを開始します" << std::endl;

  std::ifstream runFile(commandFilePath);
  if(!runFile) {
    std::cerr << "実行用コマンドファイルを開けません: " << commandFilePath << std::endl;
    return false;
  }

  // コマンド名をキーにヘッダ行（型定義）をキャッシュする（同じファイルを複数回開かないため）
  std::unordered_map<std::string, std::vector<std::string>> headerCache;

  std::string line;
  int lineNum  = 1;
  bool allValid = true;

  while(std::getline(runFile, line)) {
    // コメント削除
    size_t commentPos = line.find('#');
    if(commentPos != std::string::npos) {
      line = line.substr(0, commentPos);
    }
    trim(line);

    if(line.empty()) {
      lineNum++;
      continue;
    }

    // runParams の分割
    std::stringstream ss(line);
    std::vector<std::string> runParams;
    std::string token;
    while(std::getline(ss, token, SEPARATOR)) {
      trim(token);
      runParams.push_back(token);
    }

    if(runParams.empty()) {
      lineNum++;
      continue;
    }

    char lineBuf[32];
    snprintf(lineBuf, sizeof(lineBuf), "%d", lineNum);

    std::string commandName = runParams[0];

    // ヘッダ行をキャッシュから取得（なければファイルを開いて読む）
    if(headerCache.find(commandName) == headerCache.end()) {
      std::string motionFilePath = MOTIONS_PATH + commandName + ".csv";
      std::ifstream motionFile(motionFilePath);

      if(!motionFile) {
        std::cerr << lineBuf << "行目: 対応するMotionsファイルが存在しません: " << motionFilePath
                  << std::endl;
        allValid = false;
        lineNum++;
        continue;
      }

      std::string headerLine;
      if(!std::getline(motionFile, headerLine)) {
        std::cerr << lineBuf << "行目: Motionsファイルが空です: " << motionFilePath << std::endl;
        allValid = false;
        lineNum++;
        continue;
      }

      std::stringstream ss2(headerLine);
      std::vector<std::string> checkParams;
      while(std::getline(ss2, token, SEPARATOR)) {
        trim(token);  // ヘッダのトークンもtrimする
        checkParams.push_back(token);
      }

      headerCache[commandName] = checkParams;
    }

    const std::vector<std::string>& checkParams = headerCache[commandName];

    // 引数数チェック
    if(runParams.size() != checkParams.size()) {
      std::cerr << lineBuf << "行目 (" << commandName << "): 引数の数が一致しません"
                << " (期待: " << checkParams.size() << ", 実際: " << runParams.size() << ")"
                << std::endl;
      allValid = false;
      lineNum++;
      continue;
    }

    // 型チェック（index 0: コマンド名, index 1: ID, index 2以降: パラメータ）
    char paramBuf[32];
    for(size_t i = 2; i < checkParams.size(); i++) {
      const std::string& type  = checkParams[i];
      const std::string& value = runParams[i];
      snprintf(paramBuf, sizeof(paramBuf), "%zu", i - 1);

      if(type == "int") {
        if(!tryParse<int>(value)) {
          std::cerr << lineBuf << "行目 (" << commandName << "): 第" << paramBuf
                    << "引数はint型である必要があります (値: \"" << value << "\")" << std::endl;
          allValid = false;
        }
      } else if(type == "double") {
        if(!tryParse<double>(value)) {
          std::cerr << lineBuf << "行目 (" << commandName << "): 第" << paramBuf
                    << "引数はdouble型である必要があります (値: \"" << value << "\")" << std::endl;
          allValid = false;
        }
      } else if(type == "string") {
        // string型は任意の値を許容する
      } else {
        std::cerr << lineBuf << "行目 (" << commandName << "): 未対応の型 \"" << type << "\""
                  << std::endl;
        allValid = false;
      }
    }

    lineNum++;
  }

  return allValid;
}

int main(int argc, char* argv[])
{
  if(argc != 3) {
    std::cerr << "使い方: ./scripts/merge_commands.sh <Area名> <L|R>" << std::endl;
    std::cerr << "例: ./scripts/merge_commands.sh LineTrace L" << std::endl;
    std::cerr << "有効なArea名: ";
    for(size_t i = 0; i < AREA_NAMES.size(); i++) {
      std::cerr << AREA_NAMES[i];
      if(i + 1 < AREA_NAMES.size()) std::cerr << ", ";
    }
    std::cerr << std::endl;
    return 1;
  }

  std::string areaName  = argv[1];
  std::string courseArg = argv[2];

  if(courseArg != "L" && courseArg != "R") {
    std::cerr << "コースは L か R を指定してください (入力値: \"" << courseArg << "\")"
              << std::endl;
    return 1;
  }

  auto it = AREA_MAP.find(areaName);
  if(it == AREA_MAP.end()) {
    std::cerr << "不明なArea名: \"" << areaName << "\"" << std::endl;
    std::cerr << "有効なArea名: ";
    for(size_t i = 0; i < AREA_NAMES.size(); i++) {
      std::cerr << AREA_NAMES[i];
      if(i + 1 < AREA_NAMES.size()) std::cerr << ", ";
    }
    std::cerr << std::endl;
    return 1;
  }

  Area area           = it->second;
  bool isLeftCourse   = (courseArg == "L");

  createRunCSV(area, isLeftCourse);

  std::string course      = isLeftCourse ? "Left" : "Right";
  std::string runFilePath = RUN_PATH + "run_" + AREA_NAMES[static_cast<int>(area)] + course + ".csv";

  if(!checkType(runFilePath)) {
    std::cerr << "型チェックに失敗しました" << std::endl;
    return 1;
  }

  std::cout << "型チェックが完了しました: 問題ありません" << std::endl;
  return 0;
}
