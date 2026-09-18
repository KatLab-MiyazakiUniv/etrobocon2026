/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

#include "CameraTracking.h"
#include "LineTrace.h"
#include "RelativeAngleCondition.h"
#include "RelativeRotation.h"
#include "RepeatCountCondition.h"
#include "Snapshot.h"
#include "SocketProtocol.h"
#include "Straight.h"

#include <algorithm>

using namespace std;

// etrobocon2026/ の親ディレクトリからの実行を前提とした相対パス
static const string MOTIONS_PATH
    = "etrobocon2026/datafiles/commands/Motions/";

static const string CONDITIONS_PATH
    = "etrobocon2026/datafiles/commands/Conditions/";

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

// stringを指定した型に変換する
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

vector<BaseMotion*> MotionParser::createMotionList(
    Robot& robot,
    string& commandFilePath)
{
  int lineNum = 1;

  vector<BaseMotion*> motionList;

  // Area CSVを開く
  ifstream file(commandFilePath);

  if(!file) {
    Logger::printfLog(
        Logger::ERROR,
        "Areaのコマンドファイルを開けませんでした: %s",
        commandFilePath.c_str());

    return motionList;
  }

  string line;

  // ヘッダ行をスキップ
  if(!getline(file, line)) {
    return motionList;
  }

  lineNum++;

  while(getline(file, line)) {
    stringstream ss(line);

    vector<string> params;

    for(string token; getline(ss, token, SEPARATOR);) {
      trim(token);

      params.push_back(move(token));
    }

    if(params.size() < 4) {
      Logger::printfLog(
          Logger::ERROR,
          "%s:%d フォーマットが不正です（4列必要）",
          commandFilePath.c_str(),
          lineNum);

      lineNum++;
      continue;
    }

    string motionName = params[0];
    string motionId = params[1];

    string conditionName = params[2];
    string conditionId = params[3];

    // Motion CSVを取得
    vector<string> motionParams
        = extractParamsFromID(
            MOTIONS_PATH + motionName + ".csv",
            motionId);

    if(motionParams.empty()) {
      Logger::printfLog(
          Logger::ERROR,
          "Motions: %s ID=%s が見つかりませんでした",
          motionName.c_str(),
          motionId.c_str());

      lineNum++;
      continue;
    }

    // Condition CSVを取得
    vector<string> conditionParams
        = extractParamsFromID(
            CONDITIONS_PATH + conditionName + ".csv",
            conditionId);

    if(conditionParams.empty()) {
      Logger::printfLog(
          Logger::ERROR,
          "Conditions: %s ID=%s が見つかりませんでした",
          conditionName.c_str(),
          conditionId.c_str());

      lineNum++;
      continue;
    }

    // 条件インスタンス生成
    auto condition
        = createConditionInstance(
            robot,
            conditionParams);

    if(!condition) {
      Logger::printfLog(
          Logger::ERROR,
          "条件インスタンスの生成に失敗しました: %s %s",
          conditionName.c_str(),
          conditionId.c_str());

      lineNum++;
      continue;
    }

    // Motionインスタンス生成
    BaseMotion* motion
        = createMotionInstance(
            robot,
            motionParams,
            std::move(condition));

    if(motion) {
      motionList.push_back(motion);

      Logger::printfLog(
          Logger::INFO,
          "[MotionParser] motionList[%zu]: %s ID=%s (条件: %s ID=%s)",
          motionList.size() - 1,
          motionName.c_str(),
          motionId.c_str(),
          conditionName.c_str(),
          conditionId.c_str());
    } else {
      Logger::printfLog(
          Logger::ERROR,
          "%s:%d Command %s は未定義です",
          commandFilePath.c_str(),
          lineNum,
          motionName.c_str());
    }

    lineNum++;
  }

  return motionList;
}

vector<string> MotionParser::extractParamsFromID(
    const string& filePath,
    const string& id)
{
  ifstream file(filePath);

  if(!file) {
    Logger::printfLog(
        Logger::ERROR,
        "ファイルを開けませんでした: %s",
        filePath.c_str());

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
        Logger::printfLog(
            Logger::ERROR,
            "%s に ID=%s が重複しています",
            filePath.c_str(),
            id.c_str());

        return {};
      }

      result = move(row);
    }
  }

  return result;
}

unique_ptr<BaseContinuationCondition>
MotionParser::createConditionInstance(
    Robot& robot,
    const vector<string>& params)
{
  if(params.empty()) {
    Logger::printfLog(
        Logger::ERROR,
        "[MotionParser] 条件パラメータが空です");

    return nullptr;
  }

  CONDITION_COMMAND cond
      = convertCondition(params[0]);

  switch(cond) {

    // =============================
    // Distance
    // =============================

    case CONDITION_COMMAND::DISTANCE: {
      // params[0] = Distance
      // params[1] = ID
      // params[2] = targetDistance

      if(params.size() < 3) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] Distanceのパラメータ数が不足しています");

        return nullptr;
      }

      double targetDistance
          = fromString<double>(params[2]);

      return make_unique<DistanceCondition>(
          robot,
          targetDistance);
    }

    // =============================
    // RelativeAngle
    // =============================

    case CONDITION_COMMAND::RELATIVE_ANGLE: {
      // params[0] = RelativeAngle
      // params[1] = ID
      // params[2] = relativeAngle
      // params[3] = tolerance

      if(params.size() < 4) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] RelativeAngleのパラメータ数が不足しています");

        return nullptr;
      }

      double relativeAngle
          = fromString<double>(params[2]);

      double tolerance
          = fromString<double>(params[3]);

      return make_unique<RelativeAngleCondition>(
          robot,
          relativeAngle,
          tolerance);
    }

    // =============================
    // RepeatCount
    // =============================

    case CONDITION_COMMAND::REPEAT_COUNT: {
      // params[0] = RepeatCount
      // params[1] = ID
      // params[2] = targetCount

      if(params.size() < 3) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] RepeatCountのパラメータ数が不足しています");

        return nullptr;
      }

      int targetCount
          = fromString<int>(params[2]);

      return make_unique<RepeatCountCondition>(
          robot,
          targetCount);
    }

    default:
      Logger::printfLog(
          Logger::WARNING,
          "[MotionParser] Condition %s は未実装です",
          params[0].c_str());

      return nullptr;
  }
}

BaseMotion* MotionParser::createMotionInstance(
    Robot& robot,
    const vector<string>& motionParams,
    unique_ptr<BaseContinuationCondition> condition)
{
  if(motionParams.empty()) {
    Logger::printfLog(
        Logger::ERROR,
        "[MotionParser] 動作パラメータが空です");

    return nullptr;
  }

  MOTION_COMMAND command
      = convertCommand(motionParams[0]);

  switch(command) {

    // =============================
    // Straight
    // =============================

    case MOTION_COMMAND::STRAIGHT: {
      // motionParams[0]  = Straight
      // motionParams[1]  = ID
      // motionParams[2]  = targetSpeed
      // motionParams[3]  = rightKp
      // motionParams[4]  = rightKi
      // motionParams[5]  = rightKd
      // motionParams[6]  = leftKp
      // motionParams[7]  = leftKi
      // motionParams[8]  = leftKd
      // motionParams[9]  = angleKp
      // motionParams[10] = angleKi
      // motionParams[11] = angleKd
      // motionParams[12] = shouldUseIMU
      // motionParams[13] = deadbandRate
      // motionParams[14] = maxoutRate

      if(motionParams.size() < 15) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] Straightのパラメータ数が不足しています");

        return nullptr;
      }

      double targetSpeed
          = fromString<double>(motionParams[2]);

      Pid::PidGain rightPid{
        fromString<double>(motionParams[3]),
        fromString<double>(motionParams[4]),
        fromString<double>(motionParams[5])
      };

      Pid::PidGain leftPid{
        fromString<double>(motionParams[6]),
        fromString<double>(motionParams[7]),
        fromString<double>(motionParams[8])
      };

      Pid::PidGain anglePid{
        fromString<double>(motionParams[9]),
        fromString<double>(motionParams[10]),
        fromString<double>(motionParams[11])
      };

      bool shouldUseIMU
          = motionParams[12] == "true";

      double deadbandRate
          = fromString<double>(motionParams[13]);

      double maxoutRate
          = fromString<double>(motionParams[14]);

      return new Straight(
          robot,
          std::move(condition),
          targetSpeed,
          rightPid,
          leftPid,
          anglePid,
          shouldUseIMU,
          deadbandRate,
          maxoutRate);
    }

    // =============================
    // QRTracking
    // =============================

    case MOTION_COMMAND::QR_TRACKING: {
      if(motionParams.size() < 12) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] QRTrackingのパラメータ数が不足しています");

        return nullptr;
      }

      CameraServer::QrCodeDetectorRequest qrRequest;

      qrRequest.roi.x
          = fromString<int32_t>(motionParams[8]);

      qrRequest.roi.y
          = fromString<int32_t>(motionParams[9]);

      qrRequest.roi.width
          = fromString<int32_t>(motionParams[10]);

      qrRequest.roi.height
          = fromString<int32_t>(motionParams[11]);

      Pid::PidGain cameraPid{
        fromString<double>(motionParams[4]),
        fromString<double>(motionParams[5]),
        fromString<double>(motionParams[6])
      };

      bool isStopMotorPower
          = motionParams[7] == "true";

      return new CameraTracking(
          robot,
          std::move(condition),
          fromString<double>(motionParams[2]),
          fromString<int>(motionParams[3]),
          cameraPid,
          qrRequest,
          isStopMotorPower);
    }

    // =============================
    // LineTrace
    // =============================

    case MOTION_COMMAND::LINETRACE: {
      if(motionParams.size() < 9) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] LineTraceのパラメータ数が不足しています");

        return nullptr;
      }

      int calibratedBrightness
          = robot.getTargetBrightness();

      int targetBrightnessOffset
          = fromString<int>(motionParams[3]);

      int targetBrightness
          = std::clamp(
              calibratedBrightness
                  + targetBrightnessOffset,
              0,
              100);

      Pid::PidGain brightnessPid{
        fromString<double>(motionParams[4]),
        fromString<double>(motionParams[5]),
        fromString<double>(motionParams[6])
      };

      return new LineTrace(
          robot,
          std::move(condition),
          fromString<double>(motionParams[2]),
          targetBrightness,
          brightnessPid,
          fromString<double>(motionParams[7]),
          fromString<double>(motionParams[8]),
          motionParams[1]);
    }

    // =============================
    // RelativeRotation
    // =============================

    case MOTION_COMMAND::RELATIVE_ROTATION: {
      // motionParams[0] = RelativeRotation
      // motionParams[1] = ID
      // motionParams[2] = relativeAngle
      // motionParams[3] = angleKp
      // motionParams[4] = angleKi
      // motionParams[5] = angleKd

      if(motionParams.size() < 6) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] RelativeRotationのパラメータ数が不足しています");

        return nullptr;
      }

      double relativeAngle
          = fromString<double>(motionParams[2]);

      Pid::PidGain anglePid{
        fromString<double>(motionParams[3]),
        fromString<double>(motionParams[4]),
        fromString<double>(motionParams[5])
      };

      return new RelativeRotation(
          robot,
          std::move(condition),
          anglePid,
          relativeAngle);
    }

    // =============================
    // Snapshot
    // =============================

    case MOTION_COMMAND::SNAPSHOT: {
      // motionParams[0] = Snapshot
      // motionParams[1] = ID
      // motionParams[2] = fileName

      if(motionParams.size() < 3) {
        Logger::printfLog(
            Logger::ERROR,
            "[MotionParser] Snapshotのパラメータ数が不足しています");

        return nullptr;
      }

      return new Snapshot(
          robot,
          motionParams[2],
          std::move(condition));
    }

    default:
      Logger::printfLog(
          Logger::WARNING,
          "[MotionParser] Command %s は未実装です",
          motionParams[0].c_str());

      return nullptr;
  }
}

MotionParser::MOTION_COMMAND MotionParser::convertCommand(
    const string& str)
{
  static const unordered_map<string, MOTION_COMMAND> commandMap = {
    { "Straight", MOTION_COMMAND::STRAIGHT },
    { "QRTracking", MOTION_COMMAND::QR_TRACKING },
    { "LineTrace", MOTION_COMMAND::LINETRACE },
    { "RelativeRotation", MOTION_COMMAND::RELATIVE_ROTATION },
    { "Snapshot", MOTION_COMMAND::SNAPSHOT },
  };

  auto it = commandMap.find(str);

  if(it != commandMap.end()) {
    return it->second;
  }

  return MOTION_COMMAND::NONE;
}

MotionParser::CONDITION_COMMAND
MotionParser::convertCondition(
    const string& str)
{
  static const unordered_map<string, CONDITION_COMMAND> conditionMap = {
    { "Distance", CONDITION_COMMAND::DISTANCE },
    { "RelativeAngle", CONDITION_COMMAND::RELATIVE_ANGLE },
    { "RepeatCount", CONDITION_COMMAND::REPEAT_COUNT },
  };

  auto it = conditionMap.find(str);

  if(it != conditionMap.end()) {
    return it->second;
  }

  return CONDITION_COMMAND::NONE;
} 