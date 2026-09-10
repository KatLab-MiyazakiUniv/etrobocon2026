/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"
#include <cmath>

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
    Robot& robot, const vector<string>& params, shared_ptr<ProjectedMileage> mileage)
{
  if(!params.empty() && params[0] == "ProjectedDistance") {
    if(!mileage || params.size() != 4 || (params[2] != "X" && params[2] != "Y")) {
      Logger::error("ProjectedDistance: ETZumoFinish内で軸X/Yと目標座標を指定してください");
      return nullptr;
    }
    double target;
    try { target = fromString<double>(params[3]); }
    catch(const std::invalid_argument&) { return nullptr; }
    if(!std::isfinite(target)) return nullptr;
    auto axis = params[2] == "X" ? ProjectedDistanceCondition::Axis::HORIZONTAL
                                  : ProjectedDistanceCondition::Axis::VERTICAL;
    return make_unique<ProjectedDistanceCondition>(robot, mileage, axis, target);
  }
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
                        "[MotionParser] RepeatCountCondition: targetCount=%.1f を生成しました",
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
          "[MotionParser] DistanceAndColor: targetDistance=%.1f, targetColor=%s を生成しました",
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

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] DistanceORUltraSonic: targetDistance=%.1f, "
                        "targetSonicDistacnce=%.1f を生成しました",
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
                        "[MotionParser] UltraSonicCondition: targetDistance =%.1f を生成しました",
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
                                               unique_ptr<BaseContinuationCondition> condition,
                                               shared_ptr<ProjectedMileage> sharedMileage)
{
  // TODO: 各動作クラスが完成したら、以下のコメントを外してswitch-caseを実装する
  MOTION_COMMAND command = convertCommand(motionParams[0]);
  switch(command) {
    case MOTION_COMMAND::ET_ZUMO_FINISH: {
      // 2列目以降は動作名・動作ID・条件名・条件IDの組。
      if(motionParams.size() < 6 || (motionParams.size() - 2) % 4 != 0) return nullptr;
      auto mileage = make_shared<ProjectedMileage>();
      vector<unique_ptr<BaseMotion>> children;
      for(size_t i = 2; i < motionParams.size(); i += 4) {
        // 複合動作自身の参照による再帰を禁止する。
        if(motionParams[i] == "ETZumoFinish" || motionParams[i] == "ResetAzimuth") return nullptr;
        // 基準を設定するETZumoExitは先頭にのみ置ける。
        if(motionParams[i] == "ETZumoExit" && i != 2) return nullptr;
        auto params = extractParamsFromID(MOTIONS_PATH + motionParams[i] + ".csv", motionParams[i + 1]);
        auto conditionParams = extractParamsFromID(CONDITIONS_PATH + motionParams[i + 2] + ".csv", motionParams[i + 3]);
        if(params.empty() || conditionParams.empty()) return nullptr;
        auto childCondition = createConditionInstance(robot, conditionParams, mileage);
        if(!childCondition) return nullptr;
        if(motionParams[i] != "ETZumoExit" && conditionParams[0] != "ProjectedDistance") {
          childCondition = make_unique<ProjectedDistanceCondition>(robot, mileage, std::move(childCondition));
        }
        unique_ptr<BaseMotion> child(createMotionInstance(robot, params, std::move(childCondition), mileage));
        if(!child) return nullptr;
        children.push_back(std::move(child));
      }
      return new ETZumoFinish(robot, std::move(condition), std::move(children), mileage);
    }
    case MOTION_COMMAND::ET_ZUMO_EXIT: {
      // ETZumoExit: motionParams[0]=コマンド名
      //             motionParams[1]=動作ID
      //             motionParams[2]=0度方向への目標移動距離[mm]
      //             motionParams[3]=CameraTrackingの動作ID
      //             motionParams[4]=DistanceOrUltraSonicの条件ID
      //             motionParams[5]=AbsoluteRotationの動作ID
      //             motionParams[6]=AbsoluteAngleの条件ID
      //             motionParams[7]=Straightの動作ID

      // 必要な列数と目標距離を確認する。距離は有限の正の値のみ受け付ける。
      if(motionParams.size() != 8 && motionParams.size() != 9) {
        Logger::printfLog(Logger::ERROR,
                          "[MotionParser] ETZumoExit: 動作パラメータは8列（任意の到着色を含む場合9列）必要です（実際: %zu列）",
                          motionParams.size());
        return nullptr;
      }
      double targetDistance;
      try {
        targetDistance = fromString<double>(motionParams[2]);
      } catch(const std::invalid_argument&) {
        Logger::printfLog(Logger::ERROR,
                          "[MotionParser] ETZumoExit ID=%s: 目標距離を数値に変換できません: %s",
                          motionParams[1].c_str(), motionParams[2].c_str());
        return nullptr;
      }
      if(!std::isfinite(targetDistance) || targetDistance <= 0.0) {
        Logger::printfLog(Logger::ERROR,
                          "[MotionParser] ETZumoExit ID=%s: 目標距離は有限の正の値が必要です: %s",
                          motionParams[1].c_str(), motionParams[2].c_str());
        return nullptr;
      }

      // 全ての子動作で同じ距離計測を共有し、動作が切り替わっても積算値を引き継ぐ。
      auto mileage = sharedMileage ? sharedMileage : make_shared<ProjectedMileage>();
      vector<unique_ptr<BaseMotion>> motions;

      const string names[] = { "CameraTracking", "AbsoluteRotation", "Straight" };
      const string ids[] = { motionParams[3], motionParams[5], motionParams[7] };
      const string conditionNames[] = { "DistanceOrUltraSonic", "AbsoluteAngle" };
      const string conditionIds[] = { motionParams[4], motionParams[6] };

      for(int i = 0; i < 3; ++i) {
        // 速度やPIDゲインなどを、対応する動作CSVから取得する。
        auto params = extractParamsFromID(MOTIONS_PATH + names[i] + ".csv", ids[i]);
        if(params.empty()) {
          Logger::printfLog(
              Logger::ERROR,
              "[MotionParser] ETZumoExit ID=%s: 子動作の設定を取得できません: %s ID=%s",
              motionParams[1].c_str(), names[i].c_str(), ids[i].c_str());
          return nullptr;
        }

        // 追尾は実走行距離または超音波検知、回頭は目標角度を個別の終了条件にする。
        unique_ptr<BaseContinuationCondition> continuationCondition;
        if(i < 2) {
          auto conditionParams
              = extractParamsFromID(CONDITIONS_PATH + conditionNames[i] + ".csv", conditionIds[i]);
          if(conditionParams.empty()) {
            Logger::printfLog(
                Logger::ERROR,
                "[MotionParser] ETZumoExit ID=%s: 子動作の条件設定を取得できません: %s ID=%s",
                motionParams[1].c_str(), conditionNames[i].c_str(), conditionIds[i].c_str());
            return nullptr;
          }
          continuationCondition = createConditionInstance(robot, conditionParams);
          if(!continuationCondition) {
            Logger::printfLog(
                Logger::ERROR,
                "[MotionParser] ETZumoExit ID=%s: 継続条件の生成に失敗しました: %s ID=%s",
                motionParams[1].c_str(), conditionNames[i].c_str(), conditionIds[i].c_str());
            return nullptr;
          }
        }

        const bool requireColor = i == 2 && motionParams.size() == 9;
        if(requireColor) {
          auto color = ColorSensorController::convertStringToColor(motionParams[8]);
          if(color == ColorSensorController::COLOR::NONE) return nullptr;
          continuationCondition = make_unique<SensorColorCondition>(robot, color);
        }
        // 0度方向への積算距離が目標に達した場合は、個別の条件に関係なく終了する。
        auto projectedCondition = make_unique<ETZumoExitCondition>(
            robot, mileage, targetDistance, std::move(continuationCondition), requireColor);

        // 途中で生成に失敗しても、生成済みの子動作はunique_ptrによって解放される。
        unique_ptr<BaseMotion> motion(
            createMotionInstance(robot, params, std::move(projectedCondition)));
        if(!motion) {
          Logger::printfLog(Logger::ERROR,
                            "[MotionParser] ETZumoExit ID=%s: 子動作の生成に失敗しました: %s ID=%s",
                            motionParams[1].c_str(), names[i].c_str(), ids[i].c_str());
          return nullptr;
        }
        motions.push_back(std::move(motion));
      }

      // 子動作リストを複合動作へ渡す。
      return new ETZumoExit(robot, std::move(condition), mileage, std::move(motions));
    }
    case MOTION_COMMAND::STRAIGHT: {
      // Straight: motionParams[2]=speed(double)
      //           motionParams[9..11]=anglePid(kp,ki,kd)
      //           motionParams[12]=useIMU(string: "true"/"false")
      return new Straight(robot, std::move(condition), fromString<double>(motionParams[2]),
                          Pid::PidGain{ fromString<double>(motionParams[3]),
                                        fromString<double>(motionParams[4]),
                                        fromString<double>(motionParams[5]) },
                          motionParams[6] == "true");
    }
    case MOTION_COMMAND::LINETRACE: {
      // LineTrace: motionParams[2]=speed(double)
      //           motionParams[3]=brightness(int)
      //           motionParams[4..6]=brightnessPid(kp,ki,kd)
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
                        "[MotionParser] CameraTracking: targetXCoorddinate=%.d を生成しました",
                        targetXCoordinate);

      return new CameraTracking(
          robot, std::move(condition), fromString<double>(motionParams[2]), targetXCoordinate,
          Pid::PidGain(fromString<double>(motionParams[4]), fromString<double>(motionParams[5]),
                       fromString<double>(motionParams[6])),
          request, fromString<bool>(motionParams[7]));
    }
    case MOTION_COMMAND::ABSOLUTE_ROTATION: {
      // AbsoluteRotation:
      // motionParams[2]=anglePid.kp
      // motionParams[3]=anglePid.ki
      // motionParams[4]=anglePid.kd
      // motionParams[5]=targetAbsAngle

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
      // RelativeRotation:
      // motionParams[2]=anglePid.kp
      // motionParams[3]=anglePid.ki
      // motionParams[4]=anglePid.kd
      // motionParams[5]=relativeTargetAngle

      Pid::PidGain anglePidGain{ fromString<double>(motionParams[2]),
                                 fromString<double>(motionParams[3]),
                                 fromString<double>(motionParams[4]) };

      double relativeTargetAngle = fromString<double>(motionParams[5]);

      Logger::printfLog(Logger::DEBUG,
                        "[MotionParser] RelativeRotation: relativeTargetAngle=%.1f を生成しました",
                        relativeTargetAngle);

      return new RelativeRotation(robot, std::move(condition), anglePidGain, relativeTargetAngle);
    }
    case MOTION_COMMAND::CALIBRATOR: {
      return new Calibrator(robot, std::move(condition));
    }
    case MOTION_COMMAND::RESET_AZIMUTH: {
      return new ResetAzimuth(robot, std::move(condition));
    }
    default:
      Logger::printfLog(Logger::WARNING, "[MotionParser] Command %s は未実装です",
                        motionParams[0].c_str());
      return nullptr;
  }
}

MotionParser::MOTION_COMMAND MotionParser::convertCommand(const string& str)
{
  // コマンド文字列(string)と、それに対応する列挙型MOTION_COMMANDのマッピングを定義
  static const unordered_map<string, MOTION_COMMAND> commandMap
      = { { "Straight", MOTION_COMMAND::STRAIGHT },
          { "LineTrace", MOTION_COMMAND::LINETRACE },
          { "AbsoluteRotation", MOTION_COMMAND::ABSOLUTE_ROTATION },
          { "RelativeRotation", MOTION_COMMAND::RELATIVE_ROTATION },
          { "CameraTracking", MOTION_COMMAND::CAMERA_TRACKING },
          { "Calibrator", MOTION_COMMAND::CALIBRATOR },
          { "ResetAzimuth", MOTION_COMMAND::RESET_AZIMUTH },
          { "ETZumoExit", MOTION_COMMAND::ET_ZUMO_EXIT },
          { "ETZumoFinish", MOTION_COMMAND::ET_ZUMO_FINISH }

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
