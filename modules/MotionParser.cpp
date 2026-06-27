/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

using namespace std;

// etrobocon2026/ の親ディレクトリからの実行を前提とした相対パス
static const string MOTIONS_PATH = "etrobocon2026/datafiles/commands/Motions/";
static const string CONDITIONS_PATH = "etrobocon2026/datafiles/commands/Conditions/";

// 文字列の前後の空白を削除する
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
    Logger::printfLog(Logger::ERROR, "Areaのコマンドファイルを開けませんでした: %s",
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
      trim(token);
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

    // 動作パラメータを取得する
    vector<string> motionParams = extractParamsFromID(MOTIONS_PATH + motionName + ".csv", motionId);
    if(motionParams.empty()) {
      Logger::printfLog(Logger::ERROR, "Motions: %s ID=%s が見つかりませんでした",
                        motionName.c_str(), motionId.c_str());
      lineNum++;
      continue;
    }

    // 条件パラメータを取得する
    vector<string> conditionParams
        = extractParamsFromID(CONDITIONS_PATH + conditionName + ".csv", conditionId);
    if(conditionParams.empty()) {
      Logger::printfLog(Logger::ERROR, "Conditions: %s ID=%s が見つかりませんでした",
                        conditionName.c_str(), conditionId.c_str());
      lineNum++;
      continue;
    }
    // 条件インスタンスを生成する
    auto condition = createConditionInstance(robot, conditionParams);
    if(!condition) {
      Logger::printfLog(Logger::ERROR, "条件インスタンスの生成に失敗しました: %s %s",
                        conditionName.c_str(), conditionId.c_str());
      lineNum++;
      continue;
    }

    // 動作インスタンスを生成してリストに追加する
    BaseMotion* motion = createMotionInstance(robot, motionParams, std::move(condition));
    if(motion) {
      motionList.push_back(motion);
      Logger::printfLog(Logger::INFO, "[MotionParser] motionList[%zu]: %s ID=%s (条件: %s ID=%s)",
                        motionList.size() - 1, motionName.c_str(), motionId.c_str(),
                        conditionName.c_str(), conditionId.c_str());
    } else {
      Logger::printfLog(Logger::ERROR, "%s:%d Command %s は未定義です", commandFilePath.c_str(),
                        lineNum, motionName.c_str());
    }

    lineNum++;
  }

  return motionList;
}

vector<string> MotionParser::extractParamsFromID(const string& filePath, const string& id)
{
  ifstream file(filePath);
  if(!file) {
    Logger::printfLog(Logger::ERROR, "ファイルを開けませんでした: %s", filePath.c_str());
    return {};
  }

  // ヘッダ2行をスキップ
  string header;
  getline(file, header);
  getline(file, header);

  string line;
  vector<string> result;
  while(getline(file, line)) {
    stringstream ss(line);
    vector<string> row;
    for(string token; getline(ss, token, SEPARATOR);) {
      trim(token);
      row.push_back(move(token));
    }
    if(row.size() >= 2 && row[1] == id) {
      if(!result.empty()) {
        Logger::printfLog(Logger::ERROR, "%s に ID=%s が重複しています", filePath.c_str(),
                          id.c_str());
        return {};
      }
      result = move(row);
    }
  }

  return result;
}

unique_ptr<BaseContinuationCondition> MotionParser::createConditionInstance(
    Robot& robot, const vector<string>& params)
{
  CONDITION_COMMAND cond = convertCondition(params[0]);
  switch(cond) {
    case CONDITION_COMMAND::DISTANCE: {
      double targetDistance = fromString<double>(params[2]);
      return make_unique<DistanceCondition>(robot, targetDistance);
    }
    case CONDITION_COMMAND::ABSOLUTEANGLE: {
      double targetAngle = fromString<double>(params[2]);
      double tolerance = fromString<double>(params[3]);
      return make_unique<AbsoluteAngleCondition>(robot, targetAngle, tolerance);
    }
    case CONDITION_COMMAND::RELATIVEANGLE: {
      double targetAngle = fromString<double>(params[2]);
      double tolerance = fromString<double>(params[3]);
      return make_unique<RelativeAngleCondition>(robot, targetAngle, tolerance);
    }
    // ↓ 他の条件コマンドはここに追加していく
    default:
      Logger::printfLog(Logger::WARNING, "[MotionParser] Condition %s は未実装です",
                        params[0].c_str());
      return nullptr;
  }
}

BaseMotion* MotionParser::createMotionInstance(Robot& robot, const vector<string>& motionParams,
                                               unique_ptr<BaseContinuationCondition> condition)
{
  // TODO: 各動作クラスが完成したら、以下のコメントを外してswitch-caseを実装する
  MOTION_COMMAND command = convertCommand(motionParams[0]);
  switch(command) {
    case MOTION_COMMAND::STRAIGHT: {
      // Straight: motionParams[2]=speed(double)
      //           motionParams[3..5]=anglePid(kp,ki,kd)
      //           motionParams[6]=useIMU(string: "true"/"false")
      return new Straight(robot, std::move(condition), fromString<double>(motionParams[2]),
                          Pid::PidGain{ fromString<double>(motionParams[3]),
                                        fromString<double>(motionParams[4]),
                                        fromString<double>(motionParams[5]) },
                          motionParams[6] == "true");
    }
    case MOTION_COMMAND::ABSOLUTEROTATION: {
      // AbsoluteRotation: motionParams[2]=targetAngle(double)
      //                   motionParams[3..5]=anglePid(kp,ki,kd)
      return new AbsoluteRotation(robot, std::move(condition),
                                  Pid::PidGain{ fromString<double>(motionParams[3]),
                                                fromString<double>(motionParams[4]),
                                                fromString<double>(motionParams[5]) },
                                  fromString<double>(motionParams[2]));
    }
    case MOTION_COMMAND::RELATIVEROTATION: {
      // AbsoluteRotation: motionParams[2]=targetAngle(double)
      //                   motionParams[3..5]=anglePid(kp,ki,kd)
      return new RetativeRotation(robot, std::move(condition),
                                  Pid::PidGain{ fromString<double>(motionParams[3]),
                                                fromString<double>(motionParams[4]),
                                                fromString<double>(motionParams[5]) },
                                  fromString<double>(motionParams[2]));
    }
    // ↓ 他のコマンドはここに追加していく
    default:
      Logger::printfLog(Logger::WARNING, "[MotionParser] Command %s は未実装です",
                        motionParams[0].c_str());
      return nullptr;
  }
}

MotionParser::MOTION_COMMAND MotionParser::convertCommand(const string& str)
{
  // コマンド文字列(string)と、それに対応する列挙型MOTION_COMMANDのマッピングを定義
  static const unordered_map<string, MOTION_COMMAND> commandMap = {
    { "Straight", MOTION_COMMAND::STRAIGHT },
    { "AbsoluteRotation", MOTION_COMMAND::ABSOLUTEROTATION },
    { "RelativeRotation", MOTION_COMMAND::RELATIVEROTATION },
  };

  // コマンド文字列に対応するMOTION_COMMAND値をマップから取得。なければMOTION_COMMAND::NONEを返す
  auto it = commandMap.find(str);
  if(it != commandMap.end()) {
    return it->second;
  } else {
    return MOTION_COMMAND::NONE;
  }
}

MotionParser::CONDITION_COMMAND MotionParser::convertCondition(const string& str)
{
  // 条件コマンド文字列と、それに対応する列挙型CONDITION_COMMANDのマッピングを定義
  static const unordered_map<string, CONDITION_COMMAND> conditionMap = {
    { "Distance", CONDITION_COMMAND::DISTANCE },
    { "AbsoluteAngle", CONDITION_COMMAND::ABSOLUTEANGLE },
    { "RelativeAngle", CONDITION_COMMAND::RELATIVEANGLE },
  };

  // 条件コマンド文字列に対応するCONDITION_COMMAND値をマップから取得。なければCONDITION_COMMAND::NONEを返す
  auto it = conditionMap.find(str);
  if(it != conditionMap.end()) {
    return it->second;
  } else {
    return CONDITION_COMMAND::NONE;
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
