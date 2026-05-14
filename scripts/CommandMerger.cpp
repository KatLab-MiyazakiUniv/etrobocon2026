/**
 * @file   CommandMerger.cpp
 * @brief  AreaとMotionsのCSV結合と型チェックを行うスクリプト
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

// CSVの区切り文字
constexpr char SEPARATOR = ',';

// etrobocon2026/ ルートからの相対パス
static const std::string AREA_PATH = "datafiles/commands/Area/";
static const std::string MOTIONS_PATH = "datafiles/commands/Motions/";
static const std::string RUN_PATH = "datafiles/commands/Run/";

// Area enum のインデックスに対応するエリア名のテーブル
static const std::array<std::string, 2> AREA_NAMES = { "LineTrace", "Area2" };

// 引数文字列からArea enum の変換テーブル
static const std::unordered_map<std::string, Area> AREA_MAP = {
  { "LineTrace", LineTrace },
  { "Area2", Area2 },
};

// 文字列の前後の空白を削除するユーティリティ関数
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

// v850クロスコンパイラでは std::stoi/std::stod が利用不可のため strtol/strtod で代替する
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

// AreaのCSVファイルを読み込み、対応するMotionsのCSVファイルからコマンドを抽出し、Run用のCSVファイルを生成する
void createRunCSV(Area area, bool isLeftCourse, const std::string& label)
{
  std::string areaName = AREA_NAMES[static_cast<int>(area)];
  std::string course = isLeftCourse ? "Left" : "Right";

  std::string commandAreaFilePath = AREA_PATH + areaName + course + ".csv";
  std::string commandRunFilePath = RUN_PATH + "run_" + areaName + course + ".csv";

  std::ifstream commandAreaFile(commandAreaFilePath);
  if(!commandAreaFile) {
    std::cerr << "[" << label << "] エリアファイルを開けませんでした: " << commandAreaFilePath
              << std::endl;
    return;
  }

  std::ofstream commandRunFile(commandRunFilePath, std::ios::out | std::ios::trunc);
  if(!commandRunFile) {
    std::cerr << "[" << label << "] 実行用ファイルを開けませんでした: " << commandRunFilePath
              << std::endl;
    return;
  }

  bool firstWrite = true;  // 最初の書き込みかどうか

  std::string headerLine;
  std::getline(commandAreaFile, headerLine);

  std::string areaFileLine;
  while(std::getline(commandAreaFile, areaFileLine)) {
    size_t commentPos = areaFileLine.find('#');
    if(commentPos != std::string::npos) {
      areaFileLine = areaFileLine.substr(0, commentPos);
    }
    trim(areaFileLine);
    if(areaFileLine.empty()) continue;

    std::stringstream ssArea(areaFileLine);
    std::vector<std::string> areaFileParams;
    std::string token;
    while(std::getline(ssArea, token, SEPARATOR)) {
      trim(token);
      areaFileParams.push_back(token);
    }

    if(areaFileParams.size() != 2) {
      std::cerr << "[" << label << "] フォーマットが不正です: " << areaFileLine << std::endl;
      continue;
    }

    std::string commandName = areaFileParams[0];
    std::string commandId = areaFileParams[1];

    std::string commandMotionsFilePath = MOTIONS_PATH + commandName + ".csv";
    std::ifstream commandMotionsFile(commandMotionsFilePath);
    if(!commandMotionsFile) {
      std::cerr << "[" << label << "] Motionsファイルを開けませんでした: " << commandMotionsFilePath
                << std::endl;
      continue;
    }

    std::string motionsHeaderLine;
    std::getline(commandMotionsFile, motionsHeaderLine);
    std::getline(commandMotionsFile, motionsHeaderLine);

    bool found = false;
    std::string motionsFileLine;
    while(std::getline(commandMotionsFile, motionsFileLine)) {
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

      if(motionParams.size() >= 2 && motionParams[1] == commandId) {
        if(!firstWrite) commandRunFile << "\n";
        // motionsFileLine をそのまま書くと元ファイルの空白が残るため、
        // trim済みの motionParams をカンマで再結合して書き出す
        for(size_t j = 0; j < motionParams.size(); j++) {
          if(j > 0) commandRunFile << SEPARATOR;
          commandRunFile << motionParams[j];
        }
        firstWrite = false;
        found = true;
        break;
      }
    }

    if(!found) {
      std::cerr << "[" << label << "] " << commandName << " に ID=" << commandId
                << " が見つかりませんでした" << std::endl;
    }
  }

  commandRunFile.close();
}

// Run用のCSVファイルを読み込み、MotionsのCSVファイルのヘッダと照らし合わせて型チェックを行う
bool checkType(const std::string& commandFilePath, const std::string& label)
{
  std::ifstream runFile(commandFilePath);
  if(!runFile) {
    std::cerr << "[" << label << "] 実行用ファイルを開けません: " << commandFilePath << std::endl;
    return false;
  }

  // コマンド名をキーにヘッダ行をキャッシュし、同じMotionsファイルを複数回開くのを防ぐ
  std::unordered_map<std::string, std::vector<std::string>> headerCache;

  std::string line;      // ファイルの行を格納するバッファ
  int lineNum = 1;       // 行番号
  bool allValid = true;  // 全ての行が有効かどうかのフラグ

  while(std::getline(runFile, line)) {
    size_t commentPos = line.find('#');
    if(commentPos != std::string::npos) {
      line = line.substr(0, commentPos);
    }
    trim(line);

    if(line.empty()) {
      lineNum++;
      continue;
    }

    std::stringstream ss(line);  // 行をカンマで分割するための文字列ストリーム
    std::vector<std::string> runParams;  // 分割したパラメータを格納するベクター
    std::string token;  // 分割したトークンを一時的に格納するバッファ
    while(std::getline(ss, token, SEPARATOR)) {
      trim(token);
      runParams.push_back(token);
    }

    if(runParams.empty()) {
      lineNum++;
      continue;
    }

    char lineBuf[32];  // 行番号を文字列に変換するバッファ
    snprintf(lineBuf, sizeof(lineBuf), "%d", lineNum);

    std::string commandName = runParams[0];

    if(headerCache.find(commandName) == headerCache.end()) {
      std::string motionFilePath = MOTIONS_PATH + commandName + ".csv";
      std::ifstream motionFile(motionFilePath);

      if(!motionFile) {
        std::cerr << "[" << label << "] " << lineBuf
                  << "行目: Motionsファイルが存在しません: " << motionFilePath << std::endl;
        allValid = false;
        lineNum++;
        continue;
      }

      std::string motionHeaderLine;
      if(!std::getline(motionFile, motionHeaderLine)) {
        std::cerr << "[" << label << "] " << lineBuf
                  << "行目: Motionsファイルが空です: " << motionFilePath << std::endl;
        allValid = false;
        lineNum++;
        continue;
      }
      if(!std::getline(motionFile, motionHeaderLine)) {
        std::cerr << "[" << label << "] " << lineBuf
                  << "行目: Motionsファイルの2行目(型)が存在しません: " << motionFilePath
                  << std::endl;
        allValid = false;
        lineNum++;
        continue;
      }

      std::stringstream ssHeader(motionHeaderLine);
      std::vector<std::string> checkParams;
      while(std::getline(ssHeader, token, SEPARATOR)) {
        trim(token);
        checkParams.push_back(token);
      }

      headerCache[commandName] = checkParams;
    }

    const std::vector<std::string>& checkParams = headerCache[commandName];

    if(runParams.size() != checkParams.size()) {
      std::cerr << "[" << label << "] " << lineBuf << "行目 (" << commandName
                << "): 引数の数が一致しません" << " (期待: " << checkParams.size()
                << ", 実際: " << runParams.size() << ")" << std::endl;
      allValid = false;
      lineNum++;
      continue;
    }

    // index 0: コマンド名, index 1: ID はスキップし、index 2以降のパラメータを型チェックする
    char paramBuf[32];
    for(size_t i = 2; i < checkParams.size(); i++) {
      const std::string& type = checkParams[i];
      const std::string& value = runParams[i];
      snprintf(paramBuf, sizeof(paramBuf), "%zu", i - 1);

      if(type == "int") {
        if(!tryParse<int>(value)) {
          std::cerr << "[" << label << "] " << lineBuf << "行目 (" << commandName << "): 第"
                    << paramBuf << "引数はint型である必要があります (値: \"" << value << "\")"
                    << std::endl;
          allValid = false;
        }
      } else if(type == "double") {
        if(!tryParse<double>(value)) {
          std::cerr << "[" << label << "] " << lineBuf << "行目 (" << commandName << "): 第"
                    << paramBuf << "引数はdouble型である必要があります (値: \"" << value << "\")"
                    << std::endl;
          allValid = false;
        }
      } else if(type == "string") {
        // string型は任意の値を許容する
      } else {
        std::cerr << "[" << label << "] " << lineBuf << "行目 (" << commandName
                  << "): 未対応の型 \"" << type << "\"" << std::endl;
        allValid = false;
      }
    }

    lineNum++;
  }

  return allValid;
}

// 指定したエリアとコースに対して、CSVの結合と型チェックを行う。両方成功したらtrue、どちらか失敗したらfalseを返す。
static bool processCourse(Area area, bool isLeftCourse)
{
  std::string course = isLeftCourse ? "Left" : "Right";
  std::string label = AREA_NAMES[static_cast<int>(area)] + " " + course;

  std::cout << "[" << label << "] START CreateRunCSV" << std::endl;
  createRunCSV(area, isLeftCourse, label);
  std::cout << "[" << label << "] END CreateRunCSV" << std::endl;

  std::string runFilePath
      = RUN_PATH + "run_" + AREA_NAMES[static_cast<int>(area)] + course + ".csv";

  std::cout << "[" << label << "] START CheckType" << std::endl;
  if(!checkType(runFilePath, label)) {
    std::cout << "[" << label << "] END CheckType: FAILED" << std::endl;
    return false;
  }

  std::cout << "[" << label << "] END CheckType: OK" << std::endl;
  return true;
}

int main(int argc, char* argv[])
{
  if(argc == 1) {
    bool allValid = true;
    for(size_t i = 0; i < AREA_NAMES.size(); i++) {
      Area area = static_cast<Area>(i);
      allValid &= processCourse(area, true);
      allValid &= processCourse(area, false);
    }
    return allValid ? 0 : 1;
  }

  if(argc != 3) {
    std::cerr << "使い方: ./scripts/merge_commands.sh [<Area名> <L|R>]" << std::endl;
    std::cerr << "    例: ./scripts/merge_commands.sh LineTrace L" << std::endl;
    std::cerr << "有効なArea名: ";
    for(size_t i = 0; i < AREA_NAMES.size(); i++) {
      std::cerr << AREA_NAMES[i];
      if(i + 1 < AREA_NAMES.size()) std::cerr << ", ";
    }
    std::cerr << std::endl;
    return 1;
  }

  std::string areaName = argv[1];
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

  return processCourse(it->second, courseArg == "L") ? 0 : 1;
}
