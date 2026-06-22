/**
 * @file   CheckType.cpp
 * @brief  Area/Motions/Conditions の CSV 型チェックを行うスクリプト
 * @author nishijima515
 */

#include <array>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "../modules/Course.h"

// CSVの区切り文字
constexpr char SEPARATOR = ',';

// etrobocon2026/ ルートからの相対パス
static const std::string AREA_PATH = "datafiles/commands/Area/";
static const std::string MOTIONS_PATH = "datafiles/commands/Motions/";
static const std::string CONDITIONS_PATH = "datafiles/commands/Conditions/";

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

// 型定義行と実データ行を照合して型チェックする
static bool validateRow(const std::vector<std::string>& typeRow,
                        const std::vector<std::string>& dataRow, const std::string& cmdName,
                        const std::string& label, const std::string& locationInfo)
{
  if(dataRow.size() != typeRow.size()) {
    std::cerr << "[" << label << "] " << locationInfo << " (" << cmdName
              << "): 引数の数が一致しません (期待: " << typeRow.size()
              << ", 実際: " << dataRow.size() << ")" << std::endl;
    return false;
  }

  bool valid = true;
  char paramBuf[32];
  for(size_t i = 2; i < typeRow.size(); i++) {
    const std::string& type = typeRow[i];
    const std::string& value = dataRow[i];
    snprintf(paramBuf, sizeof(paramBuf), "%zu", i - 1);

    if(type == "int") {
      if(!tryParse<int>(value)) {
        std::cerr << "[" << label << "] " << locationInfo << " (" << cmdName << "): 第" << paramBuf
                  << "引数はint型である必要があります (値: \"" << value << "\")" << std::endl;
        valid = false;
      }
    } else if(type == "double") {
      if(!tryParse<double>(value)) {
        std::cerr << "[" << label << "] " << locationInfo << " (" << cmdName << "): 第" << paramBuf
                  << "引数はdouble型である必要があります (値: \"" << value << "\")" << std::endl;
        valid = false;
      }
    } else if(type == "string") {
      // string型は任意の値を許容する
    } else {
      std::cerr << "[" << label << "] " << locationInfo << " (" << cmdName << "): 未対応の型 \""
                << type << "\"" << std::endl;
      valid = false;
    }
  }
  return valid;
}

// 指定したCSVファイルのIDに一致する行を探して型チェックする
static bool checkRowById(const std::string& filePath, const std::string& cmdName,
                         const std::string& id, const std::string& label,
                         const std::string& locationInfo)
{
  std::ifstream file(filePath);
  if(!file) {
    std::cerr << "[" << label << "] " << locationInfo << ": ファイルが存在しません: " << filePath
              << std::endl;
    return false;
  }

  std::string h1, typeRowLine;
  if(!std::getline(file, h1) || !std::getline(file, typeRowLine)) {
    std::cerr << "[" << label << "] " << locationInfo << ": ヘッダが不足しています: " << filePath
              << std::endl;
    return false;
  }

  std::stringstream ssH(typeRowLine);
  std::vector<std::string> typeRow;
  std::string token;
  while(std::getline(ssH, token, SEPARATOR)) {
    trim(token);
    typeRow.push_back(token);
  }

  std::string line;
  while(std::getline(file, line)) {
    size_t commentPos = line.find('#');
    if(commentPos != std::string::npos) line = line.substr(0, commentPos);
    trim(line);
    if(line.empty()) continue;

    std::stringstream ss(line);
    std::vector<std::string> dataRow;
    while(std::getline(ss, token, SEPARATOR)) {
      trim(token);
      dataRow.push_back(token);
    }

    if(dataRow.size() >= 2 && dataRow[1] == id) {
      return validateRow(typeRow, dataRow, cmdName, label, locationInfo);
    }
  }

  std::cerr << "[" << label << "] " << locationInfo << " (" << cmdName << "): ID=" << id
            << " が見つかりませんでした" << std::endl;
  return false;
}

// Area CSVに書かれているコマンドのみを型チェックする
bool checkTypeForArea(const std::string& areaFilePath, const std::string& label)
{
  std::ifstream areaFile(areaFilePath);
  if(!areaFile) {
    std::cerr << "[" << label << "] エリアファイルを開けません: " << areaFilePath << std::endl;
    return false;
  }

  bool allValid = true;

  // ヘッダ行をスキップ
  std::string headerLine;
  std::getline(areaFile, headerLine);

  int lineNum = 2;
  std::string line;
  while(std::getline(areaFile, line)) {
    size_t commentPos = line.find('#');
    if(commentPos != std::string::npos) line = line.substr(0, commentPos);
    trim(line);
    if(line.empty()) {
      lineNum++;
      continue;
    }

    std::stringstream ss(line);
    std::vector<std::string> params;
    std::string token;
    while(std::getline(ss, token, SEPARATOR)) {
      trim(token);
      params.push_back(token);
    }

    if(params.size() != 4) {
      std::cerr << "[" << label << "] " << lineNum
                << "行目: フォーマットが不正です（4列必要）: " << line << std::endl;
      allValid = false;
      lineNum++;
      continue;
    }

    char lineBuf[32];
    snprintf(lineBuf, sizeof(lineBuf), "%d", lineNum);
    std::string locationInfo = std::string(lineBuf) + "行目";

    std::string motionName = params[0];
    std::string motionId = params[1];
    std::string conditionName = params[2];
    std::string conditionId = params[3];

    if(!checkRowById(MOTIONS_PATH + motionName + ".csv", motionName, motionId, label,
                     locationInfo)) {
      allValid = false;
    }
    if(!checkRowById(CONDITIONS_PATH + conditionName + ".csv", conditionName, conditionId, label,
                     locationInfo)) {
      allValid = false;
    }

    lineNum++;
  }

  return allValid;
}

// Motions/ と Conditions/ 以下の全ファイルを型チェックする
bool checkTypeAll(const std::string& label)
{
  bool allValid = true;

  auto checkDirectory = [&](const std::string& dirPath) {
    DIR* dir = opendir(dirPath.c_str());
    if(!dir) {
      std::cerr << "[" << label << "] ディレクトリを開けません: " << dirPath << std::endl;
      allValid = false;
      return;
    }

    struct dirent* entry;
    while((entry = readdir(dir)) != nullptr) {
      std::string name = entry->d_name;
      if(name.size() < 4 || name.substr(name.size() - 4) != ".csv") continue;

      std::string filePath = dirPath + name;
      std::ifstream file(filePath);
      if(!file) {
        std::cerr << "[" << label << "] ファイルを開けません: " << filePath << std::endl;
        allValid = false;
        continue;
      }

      std::string h1, typeRowLine;
      if(!std::getline(file, h1) || !std::getline(file, typeRowLine)) {
        std::cerr << "[" << label << "] ヘッダが不足しています: " << filePath << std::endl;
        allValid = false;
        continue;
      }

      std::stringstream ssH(typeRowLine);
      std::vector<std::string> typeRow;
      std::string token;
      while(std::getline(ssH, token, SEPARATOR)) {
        trim(token);
        typeRow.push_back(token);
      }

      int dataLineNum = 3;
      std::string line;
      std::unordered_map<std::string, int> seenIds;  // ID → 最初に出現した行番号
      while(std::getline(file, line)) {
        size_t commentPos = line.find('#');
        if(commentPos != std::string::npos) line = line.substr(0, commentPos);
        trim(line);
        if(line.empty()) {
          dataLineNum++;
          continue;
        }

        std::stringstream ss(line);
        std::vector<std::string> dataRow;
        while(std::getline(ss, token, SEPARATOR)) {
          trim(token);
          dataRow.push_back(token);
        }

        char lineBuf[32];
        snprintf(lineBuf, sizeof(lineBuf), "%d", dataLineNum);
        std::string cmdName = dataRow.empty() ? "" : dataRow[0];
        std::string locationInfo = filePath + ":" + lineBuf;

        // 重複IDチェック
        if(dataRow.size() >= 2) {
          const std::string& id = dataRow[1];
          auto it = seenIds.find(id);
          if(it != seenIds.end()) {
            std::cerr << "[" << label << "] " << locationInfo << " (" << cmdName << "): ID=" << id
                      << " が重複しています (最初の出現: " << it->second << "行目)" << std::endl;
            allValid = false;
          } else {
            seenIds[id] = dataLineNum;
          }
        }

        if(!validateRow(typeRow, dataRow, cmdName, label, locationInfo)) {
          allValid = false;
        }

        dataLineNum++;
      }
    }
    closedir(dir);
  };

  std::cout << "[" << label << "] START CheckTypeAll Motions" << std::endl;
  checkDirectory(MOTIONS_PATH);
  std::cout << "[" << label << "] START CheckTypeAll Conditions" << std::endl;
  checkDirectory(CONDITIONS_PATH);
  std::cout << "[" << label << "] END CheckTypeAll" << std::endl;

  return allValid;
}

// 指定したエリアとコースに対してArea CSVの型チェックを行う。成功したらtrue、失敗したらfalseを返す。
static bool processCourse(Area area, bool isLeftCourse)
{
  std::string course = isLeftCourse ? "Left" : "Right";
  std::string label = AREA_NAMES[static_cast<int>(area)] + " " + course;
  std::string areaFilePath = AREA_PATH + AREA_NAMES[static_cast<int>(area)] + course + ".csv";

  std::cout << "[" << label << "] START CheckTypeForArea" << std::endl;
  if(!checkTypeForArea(areaFilePath, label)) {
    std::cout << "[" << label << "] END CheckTypeForArea: FAILED" << std::endl;
    return false;
  }
  std::cout << "[" << label << "] END CheckTypeForArea: OK" << std::endl;
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
    allValid &= checkTypeAll("ALL");
    return allValid ? 0 : 1;
  }

  // --commands: Motions/ と Conditions/ 以下の全ファイルを型チェック
  if(argc == 2 && std::string(argv[1]) == "--commands") {
    return checkTypeAll("ALL") ? 0 : 1;
  }

  // <CSVファイルパス>: 指定した Area CSV ファイルを直接型チェック
  if(argc == 2) {
    std::string filePath = argv[1];
    std::string label = filePath;
    size_t slashPos = label.find_last_of("/\\");
    if(slashPos != std::string::npos) label = label.substr(slashPos + 1);
    size_t dotPos = label.rfind('.');
    if(dotPos != std::string::npos) label = label.substr(0, dotPos);

    std::cout << "[" << label << "] START CheckTypeForArea" << std::endl;
    if(!checkTypeForArea(filePath, label)) {
      std::cout << "[" << label << "] END CheckTypeForArea: FAILED" << std::endl;
      return 1;
    }
    std::cout << "[" << label << "] END CheckTypeForArea: OK" << std::endl;
    return 0;
  }

  if(argc != 3) {
    std::cerr << "使い方: ./scripts/check_type.sh [<CSVファイルパス>]" << std::endl;
    std::cerr << "    例: ./scripts/check_type.sh datafiles/commands/Area/LineTraceLeft.csv"
              << std::endl;
    std::cerr << "         ./scripts/check_type.sh --commands" << std::endl;
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
