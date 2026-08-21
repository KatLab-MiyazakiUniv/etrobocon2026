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

  if(!(iss >> val)) {
    throw std::invalid_argument("conversion failed");
  }

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
  if(!getline(file, line)) {
    return motionList;
  }

  lineNum++;

  // fileから1行ずつ文字列として line に読み込む
  while(getline(file, line)) {
    stringstream ss(line);

    // カンマ区切りで
    // (動作コマンド名, 動作ID, 条件コマンド名, 条件ID)
    // を取り出す
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

    case CONDITION_COMMAND::ABSOLUTE_ANGLE: {
      double targetAngle = fromString<double>(params[2]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] AbsoluteAngleCondition: targetAngle=%.1f を生成しました",
                        targetAngle);

      return make_unique<AbsoluteAngleCondition>(robot, targetAngle);
    }

    case CONDITION_COMMAND::RELATIVE_ANGLE: {
      double targetAngle = fromString<double>(params[2]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] RelativeAngleCondition: targetAngle=%.1f を生成しました",
                        targetAngle);

      return make_unique<RelativeAngleCondition>(robot, targetAngle);
    }

    case CONDITION_COMMAND::SENSOR_COLOR: {
      std::string targetColorName = params[2];

      auto targetColor = ColorSensorController::convertStringToColor(targetColorName);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] SensorColorCondition: targetColor=%s を生成しました",
                        targetColorName.c_str());

      return std::make_unique<SensorColorCondition>(robot, targetColor);
    }

    case CONDITION_COMMAND::RUNNING_TIME: {
      double targetTime = fromString<int>(params[2]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] RunningTimeCondition: targetTime=%.1f を生成しました",
                        targetTime);

      return make_unique<RunningTimeCondition>(robot, targetTime);
    }

    case CONDITION_COMMAND::MOTION_TIME: {
      double targetTime = fromString<int>(params[2]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] MotionTimeCondition: targetTime=%.1f を生成しました",
                        targetTime);

      return make_unique<MotionTimeCondition>(robot, targetTime);
    }

    case CONDITION_COMMAND::REPEAT_COUNT: {
      int targetCount = fromString<int>(params[2]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] RepeatCountCondition: targetCount=%d を生成しました",
                        targetCount);

      return make_unique<RepeatCountCondition>(robot, targetCount);
    }

    case CONDITION_COMMAND::DISTANCE_AND_COLOR: {
      double targetDistance = fromString<double>(params[2]);

      std::string targetColorName = params[3];

      auto targetColor = ColorSensorController::convertStringToColor(targetColorName);

      Logger::printfLog(
          Logger::DEBUG,
          "[MotionParser] DistanceAndColor: targetDistance=%.1f, targetColor=%s を生成しました",
          targetDistance, targetColorName.c_str());

      auto distanceCondition = std::make_unique<DistanceCondition>(robot, targetDistance);

      auto colorCondition = std::make_unique<SensorColorCondition>(robot, targetColor);

      return std::make_unique<CompoundCondition>(robot, std::move(distanceCondition),
                                                 std::move(colorCondition),
                                                 CompoundCondition::LogicalOperator::AND);
    }

    case CONDITION_COMMAND::DISTANCE_OR_COLOR: {
      double targetDistance = fromString<double>(params[2]);

      std::string targetColorName = params[3];

      auto targetColor = ColorSensorController::convertStringToColor(targetColorName);

      Logger::printfLog(
          Logger::DEBUG,
          "[MotionParser] DistanceOrColor: targetDistance=%.1f, targetColor=%s を生成しました",
          targetDistance, targetColorName.c_str());

      auto distanceCondition = std::make_unique<DistanceCondition>(robot, targetDistance);

      auto colorCondition = std::make_unique<SensorColorCondition>(robot, targetColor);

      return std::make_unique<CompoundCondition>(robot, std::move(distanceCondition),
                                                 std::move(colorCondition),
                                                 CompoundCondition::LogicalOperator::OR);
    }

    case CONDITION_COMMAND::DISTANCE_OR_ULTRA_SONIC: {
      double targetDistance = fromString<double>(params[2]);

      double targetSonicDistance = fromString<double>(params[3]);

      Logger::printfLog(
          Logger::DEBUG,
          "[MotionParser] DistanceORUltraSonic: targetDistance=%.1f, targetSonicDistance=%.1f",
          targetDistance, targetSonicDistance);

      auto distanceCondition = std::make_unique<DistanceCondition>(robot, targetDistance);

      auto ultraSonicCondition = std::make_unique<UltraSonicCondition>(robot, targetSonicDistance);

      return std::make_unique<CompoundCondition>(robot, std::move(distanceCondition),
                                                 std::move(ultraSonicCondition),
                                                 CompoundCondition::LogicalOperator::OR);
    }

    case CONDITION_COMMAND::ULTRA_SONIC: {
      double targetDistance = fromString<double>(params[2]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] UltraSonicCondition: targetDistance=%.1f を生成しました",
                        targetDistance);

      return make_unique<UltraSonicCondition>(robot, targetDistance);
    }

    default:
      Logger::printfLog(Logger::WARNING, "[MotionParser] Condition %s は未実装です",
                        params[0].c_str());

      return nullptr;
  }
}

BaseMotion* MotionParser::createMotionInstance(Robot& robot, const vector<string>& motionParams,
                                               unique_ptr<BaseContinuationCondition> condition)
{
  MOTION_COMMAND command = convertCommand(motionParams[0]);

  switch(command) {
    case MOTION_COMMAND::STRAIGHT: {
      return new Straight(robot, std::move(condition), fromString<double>(motionParams[2]),
                          Pid::PidGain{ fromString<double>(motionParams[3]),
                                        fromString<double>(motionParams[4]),
                                        fromString<double>(motionParams[5]) },
                          motionParams[6] == "true");
    }

    case MOTION_COMMAND::LINETRACE: {
      return new LineTrace(robot, std::move(condition), fromString<double>(motionParams[2]),
                           fromString<int>(motionParams[3]),
                           Pid::PidGain{ fromString<double>(motionParams[4]),
                                         fromString<double>(motionParams[5]),
                                         fromString<double>(motionParams[6]) });
    }

    case MOTION_COMMAND::CAMERA_TRACKING: {
      CameraServer::ColorRegionDetectorRequest request;

      request.requireLargestColorIndex = false;
      request.hsvRangeCount = 1;
      request.hsvRanges[0].lower = { 0, 0, 0 };
      request.hsvRanges[0].upper = { 179, 255, 30 };

      request.roi = { fromString<int>(motionParams[8]), fromString<int>(motionParams[9]),
                      fromString<int>(motionParams[10]), fromString<int>(motionParams[11]) };

      int targetXCoordinate = fromString<int>(motionParams[3]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] CameraTracking: targetXCoordinate=%d を生成しました",
                        targetXCoordinate);

      return new CameraTracking(
          robot, std::move(condition), fromString<double>(motionParams[2]), targetXCoordinate,
          Pid::PidGain(fromString<double>(motionParams[4]), fromString<double>(motionParams[5]),
                       fromString<double>(motionParams[6])),
          request, fromString<bool>(motionParams[7]));
    }

    case MOTION_COMMAND::ABSOLUTE_ROTATION: {
      Pid::PidGain anglePidGain{ fromString<double>(motionParams[2]),
                                 fromString<double>(motionParams[3]),
                                 fromString<double>(motionParams[4]) };

      double targetAbsAngle = fromString<double>(motionParams[5]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] AbsoluteRotation: targetAbsAngle=%.1f を生成しました",
                        targetAbsAngle);

      return new AbsoluteRotation(robot, std::move(condition), anglePidGain, targetAbsAngle);
    }

    case MOTION_COMMAND::RELATIVE_ROTATION: {
      Pid::PidGain anglePidGain{ fromString<double>(motionParams[2]),
                                 fromString<double>(motionParams[3]),
                                 fromString<double>(motionParams[4]) };

      double relativeTargetAngle = fromString<double>(motionParams[5]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] RelativeRotation: relativeTargetAngle=%.1f を生成しました",
                        relativeTargetAngle);

      return new RelativeRotation(robot, std::move(condition), anglePidGain, relativeTargetAngle);
    }

    case MOTION_COMMAND::EDGECHANGE: {
      std::string edge = motionParams[2];

      Logger::printfLog(Logger::DEBUG, "[MotionParser] ChangeEdge: edge=%s を生成しました",
                        edge.c_str());

      return new ChangeEdge(robot, std::move(condition), edge);
    }

    case MOTION_COMMAND::CALIBRATOR: {
      return new Calibrator(robot, std::move(condition));
    }

    case MOTION_COMMAND::SNAPSHOT: {
      std::string fileName = motionParams[2];

      Logger::printfLog(Logger::DEBUG, "[MotionParser] Snapshot: fileName=%s を生成しました",
                        fileName.c_str());

      return new Snapshot(robot, fileName, std::move(condition));
    }

    default:
      Logger::printfLog(Logger::WARNING, "[MotionParser] Command %s は未実装です",
                        motionParams[0].c_str());

      return nullptr;
  }
}

MotionParser::MOTION_COMMAND MotionParser::convertCommand(const string& str)
{
  static const unordered_map<string, MOTION_COMMAND> commandMap
      = { { "Straight", MOTION_COMMAND::STRAIGHT },
          { "LineTrace", MOTION_COMMAND::LINETRACE },
          { "AbsoluteRotation", MOTION_COMMAND::ABSOLUTE_ROTATION },
          { "RelativeRotation", MOTION_COMMAND::RELATIVE_ROTATION },
          { "EdgeChange", MOTION_COMMAND::EDGECHANGE },
          { "CameraTracking", MOTION_COMMAND::CAMERA_TRACKING },
          { "Calibrator", MOTION_COMMAND::CALIBRATOR },
          { "Snapshot", MOTION_COMMAND::SNAPSHOT } };

  auto it = commandMap.find(str);

  if(it != commandMap.end()) {
    return it->second;
  }

  return MOTION_COMMAND::NONE;
}

MotionParser::CONDITION_COMMAND MotionParser::convertCondition(const string& str)
{
  static const unordered_map<string, CONDITION_COMMAND> conditionMap
      = { { "Distance", CONDITION_COMMAND::DISTANCE },
          { "AbsoluteAngle", CONDITION_COMMAND::ABSOLUTE_ANGLE },
          { "RelativeAngle", CONDITION_COMMAND::RELATIVE_ANGLE },
          { "SensorColor", CONDITION_COMMAND::SENSOR_COLOR },
          { "RunningTime", CONDITION_COMMAND::RUNNING_TIME },
          { "MotionTime", CONDITION_COMMAND::MOTION_TIME },
          { "RepeatCount", CONDITION_COMMAND::REPEAT_COUNT },
          { "DistanceAndColor", CONDITION_COMMAND::DISTANCE_AND_COLOR },
          { "DistanceOrColor", CONDITION_COMMAND::DISTANCE_OR_COLOR },
          { "DistanceOrUltraSonic", CONDITION_COMMAND::DISTANCE_OR_ULTRA_SONIC },
          { "UltraSonic", CONDITION_COMMAND::ULTRA_SONIC } };

  auto it = conditionMap.find(str);

  if(it != conditionMap.end()) {
    return it->second;
  }

  return CONDITION_COMMAND::NONE;
}