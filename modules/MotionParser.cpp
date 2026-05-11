/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

using namespace std;

// stoi,stodの代用
// 文字列をT型に変換する関数
// 使い方
// fromString<int>("123") -> 123
// fromString<double>("1.23") -> 1.23
template <typename T>
T fromString(const std::string& s)
{
  std::istringstream iss(s);
  T val;
  if(!(iss >> val)) throw std::invalid_argument("conversion failed");
  return val;
}

// 型チェック用のstoi/stodの代替関数
// (v850クロスコンパイラはlibstdc++が簡略版のため、std::stoi/std::stodが利用不可)
template <typename T>
bool tryParse(const std::string& s);

// int型用の特化：strtolを利用して確実に判定（組み込みのistringstreamの不具合回避）
template <>
bool tryParse<int>(const std::string& s)
{
  const char* p = s.c_str();
  char* end;
  errno = 0;  // stdlib.h などのインクルードが必要だが、下部に既にあると想定
  long x = std::strtol(p, &end, 10);
  if(p == end || *end != '\0') return false;
  if(errno == ERANGE || x < INT_MIN || x > INT_MAX) return false;
  return true;
}

// double型用の特化：strtodを利用して確実に判定
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

const array<string, 2> MotionParser::areaCommandNames = { "LineTrace", "Area2" };

void MotionParser::createRunCSV(Robot& robot, Area area, bool isLeftCourse)
{
  cout << "実行用コマンドファイルの作成を開始します" << endl;

  string areaPath = "../etrobocon2026/datafiles/commands/Area/";
  string motionsPath = "../etrobocon2026/datafiles/commands/Motions/";
  string runPath = "../etrobocon2026/datafiles/commands/Run/";

  string areaName = areaCommandNames[static_cast<int>(area)];
  string course = isLeftCourse ? "Left" : "Right";

  string commandAreaFilePath = areaPath + areaName + course + ".csv";
  string commandRunFilePath = runPath + "run_" + areaName + course + ".csv";

  cout << commandAreaFilePath << endl;
  cout << commandRunFilePath << endl;

  // 文字列の先頭と末尾の空白を削除するラムダ式
  auto trim = [](string& s) {
    size_t start = s.find_first_not_of(" \t");
    if(start == string::npos) {
      s.clear();
      return;
    }
    size_t end = s.find_last_not_of(" \t");
    s = s.substr(start, end - start + 1);
  };

  // 入力ファイルを読み取りモードで開く
  ifstream commandAreaFile(commandAreaFilePath);
  if(!commandAreaFile) {
    cerr << "エリアコマンドファイルを開けませんでした: " << commandAreaFilePath << endl;
    return;
  }

  // 出力ファイルを書き込みモードで開き、初期化する
  ofstream commandRunFile(commandRunFilePath, ios::out | ios::trunc);
  if(!commandRunFile) {
    cerr << "実行用コマンドファイルを開けませんでした: " << commandRunFilePath << endl;
    return;
  }

  // 改行制御フラグ
  bool firstWrite = true;

  // 1行目（カラム名）をスキップ
  string headerLine;
  getline(commandAreaFile, headerLine);

  string areaFileLine;

  while(getline(commandAreaFile, areaFileLine)) {
    // コメント削除
    size_t areaPos = areaFileLine.find('#');
    if(areaPos != string::npos) {
      areaFileLine = areaFileLine.substr(0, areaPos);
    }

    // trim（行全体）
    trim(areaFileLine);

    // 空行スキップ
    if(areaFileLine.empty()) continue;

    // CSV分割
    stringstream ssArea(areaFileLine);
    vector<string> areaFileParams;
    string token;

    while(getline(ssArea, token, SEPARATOR)) {
      trim(token);
      areaFileParams.push_back(token);
    }

    // 列数チェック
    if(areaFileParams.size() != 2) {
      cerr << "エリアコマンドファイルのフォーマットが不正です: " << areaFileLine << endl;
      continue;
    }

    string commandName = areaFileParams[0];
    string commandID = areaFileParams[1];

    string commandMotionsFilePath = motionsPath + commandName + ".csv";

    // 中間ファイル(Motionごとのコマンドファイル)の読み込み
    ifstream commandMotionsFile(commandMotionsFilePath);
    if(!commandMotionsFile) {
      cerr << "動作コマンドファイルを開けませんでした: " << commandMotionsFilePath << endl;
      continue;
    }

    // 1行目（カラム名）をスキップ
    string motionsHeaderLine;
    getline(commandMotionsFile, motionsHeaderLine);

    string motionsFileLine;

    while(getline(commandMotionsFile, motionsFileLine)) {
      // コメント削除
      size_t motionsPos = motionsFileLine.find('#');
      if(motionsPos != string::npos) {
        motionsFileLine = motionsFileLine.substr(0, motionsPos);
      }

      // trim
      trim(motionsFileLine);

      if(motionsFileLine.empty()) continue;

      // CSV分割
      stringstream ssMotions(motionsFileLine);
      vector<string> motionParams;

      while(getline(ssMotions, token, SEPARATOR)) {
        trim(token);
        motionParams.push_back(token);
      }

      // cout << "比較: commandID vs motionID = [ " << commandID << " ] vs [ " << motionParams[1] <<
      // " ]" << endl;

      // ID一致チェック（2列目がID想定）
      if(motionParams.size() >= 2 && motionParams[1] == commandID) {
        // 1行目以外は改行してから書き込む
        if(!firstWrite) {
          commandRunFile << "\n";
        }

        commandRunFile << motionsFileLine;
        firstWrite = false;

        // commandRunFile << motionsFileLine << "\n";
        break;
      }
    }
  }

  cout << "実行用コマンドファイルの作成が完了しました" << endl;

  commandRunFile.close();  // バッファを書き出してファイルを閉じる

  checkType(commandRunFilePath);
}

bool MotionParser::checkType(string& commandFilePath)
{
  string areaPath = "../etrobocon2026/datafiles/commands/Area/";
  string motionsPath = "../etrobocon2026/datafiles/commands/Motions/";
  string runPath = "../etrobocon2026/datafiles/commands/Run/";

  // 文字列の先頭と末尾の空白を削除するラムダ式
  auto trim = [](string& s) {
    size_t start = s.find_first_not_of(" \t");
    if(start == string::npos) {
      s.clear();
      return;
    }
    size_t end = s.find_last_not_of(" \t");
    s = s.substr(start, end - start + 1);
  };

  // 1. 指定されたファイルパスのファイルを開く
  cout << "型チェックを開始します" << endl;
  ifstream runFile(commandFilePath);
  if(!runFile) {
    cerr << "実行用コマンドファイルを開けません: " << commandFilePath << endl;
    return false;
  }

  string line;
  int lineNum = 1;
  bool allValid = true;

  while(getline(runFile, line)) {
    // コメント除去
    size_t pos = line.find('#');
    if(pos != string::npos) {
      line = line.substr(0, pos);
    }

    // trim
    trim(line);

    if(line.empty()) {
      lineNum++;
      continue;
    }

    // --- runParams ---
    stringstream ss(line);
    vector<string> runParams;
    string token;

    while(getline(ss, token, SEPARATOR)) {
      trim(token);
      runParams.push_back(token);
    }

    if(runParams.empty()) {
      lineNum++;
      continue;
    }

    string commandName = runParams[0];

    // --- Motions CSVを開く ---
    string motionFilePath = motionsPath + commandName + ".csv";
    ifstream motionFile(motionFilePath);

    char lineBuf[32];
    sprintf(lineBuf, "%d", lineNum);

    if(!motionFile) {
      cerr << lineBuf << "行目: 対応するMotionsファイルが存在しません: " << motionFilePath << endl;
      allValid = false;
      lineNum++;
      continue;
    }

    // --- 1行目（定義）取得 ---
    string headerLine;
    if(!getline(motionFile, headerLine)) {
      cerr << lineBuf << "行目: Motionsファイルが空です" << endl;
      allValid = false;
      lineNum++;
      continue;
    }

    // --- checkParams ---
    stringstream ss2(headerLine);
    vector<string> checkParams;

    while(getline(ss2, token, SEPARATOR)) {
      checkParams.push_back(token);
    }

    // --- 引数数チェック ---
    if(runParams.size() != checkParams.size()) {
      cerr << lineBuf << "行目 (" << commandName << "): 引数の数が一致しません" << endl;
      allValid = false;
      lineNum++;
      continue;
    }

    char paramBuf[32];

    // --- 型チェック ---
    for(size_t i = 2; i < checkParams.size(); i++) {
      string type = checkParams[i];
      string value = runParams[i];
      sprintf(paramBuf, "%d", i - 1);
      if(type == "int") {
        if(!tryParse<int>(value)) {
          cerr << lineBuf << "行目 (" << commandName << "): 第" << paramBuf
               << "引数はint型である必要があります" << endl;
          allValid = false;
        }
      } else if(type == "double") {
        if(!tryParse<double>(value)) {
          cerr << lineBuf << "行目 (" << commandName << "): 第" << paramBuf
               << "引数はdouble型である必要があります" << endl;
          allValid = false;
        }
      } else if(type == "string") {
        // OK（必要なら制約追加）
      } else {
        cerr << lineBuf << "行目 (" << commandName << "): 未対応の型 " << type << endl;
        allValid = false;
      }
    }

    lineNum++;
  }

  return allValid;
}

vector<BaseMotion*> MotionParser::createMotionList(Robot& robot, string& commandFilePath)
{
  // 行番号カウンタ
  int lineNum = 1;
  // 動作インスタンスのリスト
  vector<BaseMotion*> motionList;

  // ファイルを開き、開けなければ空のリストを返す
  ifstream file(commandFilePath);
  if(!file) {
    cout << "コマンドファイルを開けませんでした: " << commandFilePath << endl;
    return motionList;
  }

  // 各行を格納する変数を定義
  string line;

  // fileから1行ずつ文字列として line に読み込む
  // TODO: 各動作クラスが完成したら、以下のコメントを外してswitch-caseを実装する
  while(getline(file, line)) {
    // 文字列 line をストリームに変換
    stringstream ss(line);

    // カンマ区切りでコマンド名とその引数を1つずつ取り出して params に追加
    vector<string> params;
    for(string token; getline(ss, token, SEPARATOR);) {
      params.push_back(move(token));
    }

    // TODO: コマンド名をCOMMAND enumに変換して各Motionオブジェクトを生成する処理
    //       各コマンドに対応するMotionクラスが実装済みになったら順次追加していく
    // COMMAND command = convertCommand(params[0]);
    // switch(command) {
    //   case COMMAND::AR: {
    //     // AR: 角度指定回頭
    //     // params[1]:int 角度[deg], params[2]:double 速度[mm/s], params[3]:string 方向
    //     auto ar = new AngleRotation(robot, std::stoi(params[1]), std::stod(params[2]),
    //                                 convertBool(params[0], params[3]));
    //     motionList.push_back(ar);
    //     break;
    //   }
    //   // ↓ 他のコマンドはここに追加していく
    //   default: {
    //     cout << commandFilePath << ":" << lineNum << " Command " << params[0] << " は未定義です"
    //     << endl; break;
    //   }
    // }

    lineNum++;  // 行番号をインクリメントする
  }

  return motionList;
}

COMMAND MotionParser::convertCommand(const string& str)
{
  // コマンド文字列(string)と、それに対応する列挙型COMMANDのマッピングを定義
  static const unordered_map<string, COMMAND> commandMap = {
    { "EXAMPLE", COMMAND::EXAMPLE },  // 例
  };

  // コマンド文字列に対応するCOMMAND値をマップから取得。なければCOMMAND::NONEを返す
  auto it = commandMap.find(str);
  if(it != commandMap.end()) {
    return it->second;
  } else {
    return COMMAND::NONE;
  }
}

// bool MotionParser::convertBool(const string& command, const string& stringParameter)
// {
//   // 末尾の改行を削除
//   string param = StringOperator::removeEOL(stringParameter);

//   // カメラPIDトラッキング系の停止制御（continueなら継続、stopなら停止）
//   if(command == "DCL" || command == "CDCL" || command == "UDCL" || command == "DTCCL"
//      || command == "CDTCCL") {
//     if(param == "continue") {
//       return false;
//     } else if(param == "stop") {
//       return true;
//     } else {
//       cout << "'continue' か 'stop'を入力してください" << endl;
//       return true;
//     }
//   }

//   //
//   回転動作(AR,IMUR,MCA,BCA)の場合、"clockwise"ならtrue（時計回り）、"anticlockwise"ならfalse（反時計回り）に変換
//   if(command == "AR" || command == "IMUR" || command == "MCA" || command == "BCA"
//      || command == "CRA") {
//     if(param == "clockwise") {
//       return true;
//     } else if(param == "anticlockwise") {
//       return false;
//     } else {
//       cout << "'clockwise' か 'anticlockwise'を入力してください" << endl;
//       return true;
//     }
//   }

//   // エッジ切り替え(EC)の場合、"left"ならtrue（左エッジ）、"right"ならfalse（右エッジ)に変換
//   if(command == "EC") {
//     if(param == "left") {
//       return true;
//     } else if(param == "right") {
//       return false;
//     } else {
//       cout << "'left' か 'right'を入力してください" << endl;
//       return true;
//     }
//   }

//   // IMU設定(IS)の場合、"start"ならtrue（開始）、"stop"ならfalse（停止)に変換
//   if(command == "IS") {
//     if(param == "start") {
//       return true;
//     } else if(param == "stop") {
//       return false;
//     } else {
//       cout << "'start' か 'stop'を入力してください" << endl;
//       return false;
//     }
//   }

//   // ここまでに条件を満たしていなかった場合は、デフォルト値としてtrueを返す
//   cout << "convertBool関数の処理の対象外です: '" << command << endl;
//   return true;
// }

// bool MotionParser::convertRotationModeToBool(const string& stringParameter)
// {
//   // 末尾の改行を削除
//   string param = StringOperator::removeEOL(stringParameter);

//   // "relative"ならfalse（相対角度回頭）、"absolute"ならtrue（絶対角度回頭）に変換
//   if(param == "relative") {
//     return false;
//   } else if(param == "absolute") {
//     return true;
//   } else {
//     cout << "'relative' か 'absolute'を入力してください (入力値: " << param << ")" << endl;
//     return false;  // デフォルトは相対角度回頭
//   }
// }