/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"
#include "DistanceCondition.h"

using namespace std;

// etrobocon2026/ の親ディレクトリからの実行を前提とした相対パス
static const string MOTIONS_PATH = "etrobocon2026/datafiles/commands/Motions/";
static const string CONDITIONS_PATH = "etrobocon2026/datafiles/commands/Conditions/";

// stringを指定した型に変換する関数(stoi,stodの代わり)
template <typename T>
T fromString(const std::string& s)
{
  std::istringstream iss(s);
  T val;
  if(!(iss >> val)) throw std::invalid_argument("conversion failed");
  return val;
}

vector<BaseMotion*> MotionParser::createMotionList(Robot& robot, string& commandFilePath)
{
  int lineNum = 1;
  vector<BaseMotion*> motionList;

  // Area CSVを開き、開けなければ空のリストを返す
  ifstream file(commandFilePath);
  if(!file) {
    Logger::printfLog(Logger::ERROR, "コマンドファイルを開けませんでした: %s",
                      commandFilePath.c_str());
    return motionList;
  }

  string line;

  // ヘッダ行をスキップ
  if(!getline(file, line)) return motionList;
  lineNum++;

  // fileから1行ずつ文字列として line に読み込む
  while(getline(file, line)) {
    stringstream ss(line);

    // カンマ区切りで (動作コマンド名, 動作ID, 条件コマンド名, 条件ID) を取り出す
    vector<string> params;
    for(string token; getline(ss, token, SEPARATOR);) {
      params.push_back(move(token));
    }

    if(params.size() < 4) {
      Logger::printfLog(Logger::ERROR, "%s:%d フォーマットが不正です（4列必要）",
                        commandFilePath.c_str(), lineNum);
      lineNum++;
      continue;
    }

    string motionName = params[0];
    string motionId = params[1];
    string conditionName = params[2];
    string conditionId = params[3];

    // Motions/<motionName>.csv を開いて motionId の行を検索する
    string motionsFilePath = MOTIONS_PATH + motionName + ".csv";
    ifstream motionsFile(motionsFilePath);
    if(!motionsFile) {
      Logger::printfLog(Logger::ERROR, "Motionsファイルを開けませんでした: %s",
                        motionsFilePath.c_str());
      lineNum++;
      continue;
    }

    // ヘッダ2行をスキップ
    string motionsHeader;
    getline(motionsFile, motionsHeader);
    getline(motionsFile, motionsHeader);

    vector<string> motionParams;
    bool motionFound = false;
    string motionsLine;
    while(getline(motionsFile, motionsLine)) {
      stringstream ssMotions(motionsLine);
      vector<string> row;
      for(string token; getline(ssMotions, token, SEPARATOR);) {
        row.push_back(move(token));
      }
      if(row.size() >= 2 && row[1] == motionId) {
        motionParams = move(row);
        motionFound = true;
        break;
      }
    }

    if(!motionFound) {
      Logger::printfLog(Logger::ERROR, "%s に ID=%s が見つかりませんでした",
                        motionsFilePath.c_str(), motionId.c_str());
      lineNum++;
      continue;
    }
    Logger::printfLog(Logger::DEBUG, "[MotionParser] Motion: %s ID=%s をロードしました",
                      motionName.c_str(), motionId.c_str());
    {
      string paramsDebug;
      for(size_t i = 0; i < motionParams.size(); i++) {
        if(i > 0) paramsDebug += ", ";
        paramsDebug += "[" + to_string(i) + "]=" + motionParams[i];
      }
      Logger::printfLog(Logger::DEBUG, "[MotionParser] Motion params: %s", paramsDebug.c_str());
    }

    // 条件インスタンスを生成する
    auto condition = createConditionList(robot, conditionName, conditionId);
    if(!condition) {
      Logger::printfLog(Logger::ERROR, "条件インスタンスの生成に失敗しました: %s %s",
                        conditionName.c_str(), conditionId.c_str());
      lineNum++;
      continue;
    }
    Logger::printfLog(
        Logger::DEBUG, "[MotionParser] Condition: %s ID=%s を Motion: %s ID=%s に注入します",
        conditionName.c_str(), conditionId.c_str(), motionName.c_str(), motionId.c_str());

    // TODO: 各動作クラスが完成したら、以下のコメントを外してswitch-caseを実装する
    COMMAND command = convertCommand(motionName);
    switch(command) {
      // case COMMAND::STRAIGHT: {
      //   // Straight: motionParams[2]=speed(double)
      //   //           motionParams[3..5]=rightPid(kp,ki,kd)
      //   //           motionParams[6..8]=leftPid(kp,ki,kd)
      //   //           motionParams[9..11]=anglePid(kp,ki,kd)
      //   //           motionParams[12]=useIMU(string: "true"/"false")
      //   auto straight = new Straight(
      //       robot, std::move(condition),
      //       fromString<double>(motionParams[2]),
      //       Pid::PidGain{ fromString<double>(motionParams[3]),
      //                     fromString<double>(motionParams[4]),
      //                     fromString<double>(motionParams[5]) },
      //       Pid::PidGain{ fromString<double>(motionParams[6]),
      //                     fromString<double>(motionParams[7]),
      //                     fromString<double>(motionParams[8]) },
      //       Pid::PidGain{ fromString<double>(motionParams[9]),
      //                     fromString<double>(motionParams[10]),
      //                     fromString<double>(motionParams[11]) },
      //       motionParams[12] == "true");
      //   motionList.push_back(straight);
      //   break;
      // }
      // ↓ 他のコマンドはここに追加していく
      default: {
        Logger::printfLog(Logger::ERROR, "%s:%d Command %s は未定義です", commandFilePath.c_str(),
                          lineNum, motionName.c_str());
        break;
      }
    }

    lineNum++;
  }

  return motionList;
}

unique_ptr<BaseContinuationCondition> MotionParser::createConditionList(Robot& robot,
                                                                        const string& conditionName,
                                                                        const string& conditionId)
{
  string conditionsFilePath = CONDITIONS_PATH + conditionName + ".csv";
  ifstream file(conditionsFilePath);
  if(!file) {
    Logger::printfLog(Logger::ERROR, "Conditionsファイルを開けませんでした: %s",
                      conditionsFilePath.c_str());
    return nullptr;
  }

  // ヘッダ2行をスキップ
  string header;
  getline(file, header);
  getline(file, header);

  string line;
  while(getline(file, line)) {
    stringstream ss(line);
    vector<string> params;
    for(string token; getline(ss, token, SEPARATOR);) {
      params.push_back(move(token));
    }

    if(params.size() >= 2 && params[1] == conditionId) {
      CONDITION cond = convertCondition(params[0]);
      switch(cond) {
        case CONDITION::DISTANCE: {
          double targetDistance = fromString<double>(params[2]);
          Logger::printfLog(Logger::DEBUG,
                            "[MotionParser] DistanceCondition: targetDistance=%.1f を生成しました",
                            targetDistance);
          return make_unique<DistanceCondition>(robot, targetDistance);
        }
        default: {
          Logger::printfLog(Logger::ERROR, "Condition %s は未定義です", conditionName.c_str());
          return nullptr;
        }
      }
    }
  }

  Logger::printfLog(Logger::ERROR, "%s に ID=%s が見つかりませんでした", conditionsFilePath.c_str(),
                    conditionId.c_str());
  return nullptr;
}

COMMAND MotionParser::convertCommand(const string& str)
{
  // コマンド文字列(string)と、それに対応する列挙型COMMANDのマッピングを定義
  static const unordered_map<string, COMMAND> commandMap = {
    { "EXAMPLE", COMMAND::EXAMPLE },
    { "Straight", COMMAND::STRAIGHT },
  };

  // コマンド文字列に対応するCOMMAND値をマップから取得。なければCOMMAND::NONEを返す
  auto it = commandMap.find(str);
  if(it != commandMap.end()) {
    return it->second;
  } else {
    return COMMAND::NONE;
  }
}

CONDITION MotionParser::convertCondition(const string& str)
{
  // 条件コマンド文字列と、それに対応する列挙型CONDITIONのマッピングを定義
  static const unordered_map<string, CONDITION> conditionMap = {
    { "Distance", CONDITION::DISTANCE },
  };

  // 条件コマンド文字列に対応するCONDITION値をマップから取得。なければCONDITION::NONEを返す
  auto it = conditionMap.find(str);
  if(it != conditionMap.end()) {
    return it->second;
  } else {
    return CONDITION::NONE;
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
