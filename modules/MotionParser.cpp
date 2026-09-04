/**
 * @file   MotionParser.cpp
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#include "MotionParser.h"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "CameraTracking.h"
#include "Logger.h"
#include "SocketProtocol.h"
#include "Straight.h"

using namespace std;

// etrobocon2026/ の親ディレクトリからの実行を前提とした相対パス
static const string MOTIONS_PATH =
    "etrobocon2026/datafiles/commands/Motions/";

static const string CONDITIONS_PATH =
    "etrobocon2026/datafiles/commands/Conditions/";

/**
 * @brief 文字列の前後の空白を削除する
 */
static void trim(string& str)
{
  const size_t start =
      str.find_first_not_of(" \t\r\n");

  if(start == string::npos) {
    str.clear();
    return;
  }

  const size_t end =
      str.find_last_not_of(" \t\r\n");

  str =
      str.substr(
          start,
          end - start + 1);
}

/**
 * @brief stringを指定した型に変換する
 */
template <typename T>
T fromString(const string& str)
{
  istringstream iss(str);

  T value;

  if(!(iss >> value)) {
    throw invalid_argument(
        "conversion failed: " + str);
  }

  return value;
}

/**
 * @brief AreaコマンドCSVから動作リストを生成する
 */
vector<BaseMotion*> MotionParser::createMotionList(
    Robot& robot,
    string& commandFilePath)
{
  int lineNum = 1;

  vector<BaseMotion*> motionList;

  // =========================================================
  // Area CSVを開く
  // =========================================================

  ifstream file(commandFilePath);

  if(!file) {
    Logger::printfLog(
        Logger::ERROR,
        "Areaのコマンドファイルを開けませんでした: %s",
        commandFilePath.c_str());

    return motionList;
  }

  string line;

  // =========================================================
  // Area CSVのヘッダ1行をスキップ
  // =========================================================

  if(!getline(file, line)) {
    return motionList;
  }

  lineNum++;

  // =========================================================
  // Area CSVを1行ずつ読み込む
  // =========================================================

  while(getline(file, line)) {

    // 空行は無視
    if(line.empty()) {
      lineNum++;
      continue;
    }

    stringstream ss(line);

    vector<string> params;

    for(string token;
        getline(ss, token, SEPARATOR);) {

      trim(token);

      params.push_back(
          move(token));
    }

    // =======================================================
    // Area CSVは4列必要
    //
    // motionName
    // motionId
    // conditionName
    // conditionId
    // =======================================================

    if(params.size() < 4) {
      Logger::printfLog(
          Logger::ERROR,
          "%s:%d フォーマットが不正です（4列必要）",
          commandFilePath.c_str(),
          lineNum);

      lineNum++;

      continue;
    }

    const string motionName =
        params[0];

    const string motionId =
        params[1];

    const string conditionName =
        params[2];

    const string conditionId =
        params[3];

    // =======================================================
    // Motion CSVから動作パラメータ取得
    // =======================================================

    vector<string> motionParams =
        extractParamsFromID(
            MOTIONS_PATH
                + motionName
                + ".csv",
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

    // =======================================================
    // Condition CSVから条件パラメータ取得
    // =======================================================

    vector<string> conditionParams =
        extractParamsFromID(
            CONDITIONS_PATH
                + conditionName
                + ".csv",
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

    // =======================================================
    // 継続条件生成
    // =======================================================

    unique_ptr<BaseContinuationCondition> condition =
        createConditionInstance(
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

    // =======================================================
    // Motion生成
    // =======================================================

    BaseMotion* motion =
        createMotionInstance(
            robot,
            motionParams,
            move(condition));

    if(motion != nullptr) {

      motionList.push_back(
          motion);

      Logger::printfLog(
          Logger::INFO,
          "[MotionParser] motionList[%zu]: "
          "%s ID=%s "
          "(条件: %s ID=%s)",
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

/**
 * @brief 指定されたCSVからIDに対応する行を取得する
 */
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

  // =========================================================
  // Motion / Condition CSVはヘッダ2行をスキップ
  // =========================================================

  string header;

  getline(file, header);
  getline(file, header);

  string line;

  vector<string> result;

  while(getline(file, line)) {

    if(line.empty()) {
      continue;
    }

    stringstream ss(line);

    vector<string> row;

    for(string token;
        getline(ss, token, SEPARATOR);) {

      trim(token);

      row.push_back(
          move(token));
    }

    // IDは2列目
    if(row.size() >= 2
       && row[1] == id) {

      // ID重複
      if(!result.empty()) {

        Logger::printfLog(
            Logger::ERROR,
            "%s に ID=%s が重複しています",
            filePath.c_str(),
            id.c_str());

        return {};
      }

      result =
          move(row);
    }
  }

  return result;
}

/**
 * @brief 継続条件を生成する
 */
unique_ptr<BaseContinuationCondition>
MotionParser::createConditionInstance(
    Robot& robot,
    const vector<string>& params)
{
  if(params.empty()) {
    Logger::error(
        "[MotionParser] Conditionパラメータが空です");

    return nullptr;
  }

  CONDITION_COMMAND conditionCommand =
      convertCondition(
          params[0]);

  switch(conditionCommand) {

    // =======================================================
    // Distance
    // =======================================================

    case CONDITION_COMMAND::DISTANCE: {

      if(params.size() < 3) {
        Logger::error(
            "[MotionParser] Distanceのパラメータ数が不足しています");

        return nullptr;
      }

      const double targetDistance =
          fromString<double>(
              params[2]);

      Logger::printfLog(
          Logger::INFO,
          "[MotionParser] Distance "
          "targetDistance=%.2f",
          targetDistance);

      return make_unique<DistanceCondition>(
          robot,
          targetDistance);
    }

    // =======================================================
    // 未定義
    // =======================================================

    default:

      Logger::printfLog(
          Logger::WARNING,
          "[MotionParser] Condition %s は未実装です",
          params[0].c_str());

      return nullptr;
  }
}

/**
 * @brief Motionを生成する
 */
BaseMotion* MotionParser::createMotionInstance(
    Robot& robot,
    const vector<string>& motionParams,
    unique_ptr<BaseContinuationCondition> condition)
{
  if(motionParams.empty()) {
    Logger::error(
        "[MotionParser] Motionパラメータが空です");

    return nullptr;
  }

  MOTION_COMMAND command =
      convertCommand(
          motionParams[0]);

  switch(command) {

    // =======================================================
    // Straight
    //
    // Straight内部のSpeedCalculatorへ
    // rightPid / leftPid が渡される
    // =======================================================

    case MOTION_COMMAND::STRAIGHT: {

      /*
       * Straight.csv
       *
       * [0]  command
       * [1]  id
       * [2]  speed
       *
       * SpeedCalculator 右PID
       * [3]  rightKp
       * [4]  rightKi
       * [5]  rightKd
       *
       * SpeedCalculator 左PID
       * [6]  leftKp
       * [7]  leftKi
       * [8]  leftKd
       *
       * IMU角度補正PID
       * [9]  angleKp
       * [10] angleKi
       * [11] angleKd
       *
       * [12] shouldUseIMU
       */

      if(motionParams.size() < 13) {
        Logger::error(
            "[MotionParser] Straightのパラメータ数が不足しています");

        return nullptr;
      }

      const double targetSpeed =
          fromString<double>(
              motionParams[2]);

      const Pid::PidGain rightPid = {
          fromString<double>(
              motionParams[3]),

          fromString<double>(
              motionParams[4]),

          fromString<double>(
              motionParams[5])
      };

      const Pid::PidGain leftPid = {
          fromString<double>(
              motionParams[6]),

          fromString<double>(
              motionParams[7]),

          fromString<double>(
              motionParams[8])
      };

      const Pid::PidGain anglePid = {
          fromString<double>(
              motionParams[9]),

          fromString<double>(
              motionParams[10]),

          fromString<double>(
              motionParams[11])
      };

      const bool shouldUseIMU =
          motionParams[12] == "true";

      Logger::printfLog(
          Logger::INFO,
          "[MotionParser] Straight "
          "speed=%.2f "
          "rightPID=(%.6f,%.6f,%.6f) "
          "leftPID=(%.6f,%.6f,%.6f) "
          "anglePID=(%.6f,%.6f,%.6f) "
          "useIMU=%s",
          targetSpeed,
          rightPid.kp,
          rightPid.ki,
          rightPid.kd,
          leftPid.kp,
          leftPid.ki,
          leftPid.kd,
          anglePid.kp,
          anglePid.ki,
          anglePid.kd,
          shouldUseIMU
              ? "true"
              : "false");

      return new Straight(
          robot,
          move(condition),
          targetSpeed,
          rightPid,
          leftPid,
          anglePid,
          shouldUseIMU);
    }

    // =======================================================
    // QRTracking
    // =======================================================

    case MOTION_COMMAND::QR_TRACKING: {

      /*
       * QRTracking.csv
       *
       * [0]  command
       * [1]  id
       * [2]  speed
       * [3]  targetX
       * [4]  kp
       * [5]  ki
       * [6]  kd
       * [7]  isStopMotorPower
       * [8]  roiX
       * [9]  roiY
       * [10] roiWidth
       * [11] roiHeight
       */

      if(motionParams.size() < 12) {
        Logger::error(
            "[MotionParser] QRTrackingのパラメータ数が不足しています");

        return nullptr;
      }

      CameraServer::QrCodeDetectorRequest qrRequest {};

      qrRequest.roi.x =
          fromString<int32_t>(
              motionParams[8]);

      qrRequest.roi.y =
          fromString<int32_t>(
              motionParams[9]);

      qrRequest.roi.width =
          fromString<int32_t>(
              motionParams[10]);

      qrRequest.roi.height =
          fromString<int32_t>(
              motionParams[11]);

      const double targetSpeed =
          fromString<double>(
              motionParams[2]);

      const int targetX =
          fromString<int>(
              motionParams[3]);

      const Pid::PidGain cameraPid = {
          fromString<double>(
              motionParams[4]),

          fromString<double>(
              motionParams[5]),

          fromString<double>(
              motionParams[6])
      };

      const bool isStopMotorPower =
          motionParams[7] == "true";

      Logger::printfLog(
          Logger::INFO,
          "[MotionParser] QRTracking "
          "speed=%.2f targetX=%d",
          targetSpeed,
          targetX);

      return new CameraTracking(
          robot,
          move(condition),
          targetSpeed,
          targetX,
          cameraPid,
          qrRequest,
          isStopMotorPower);
    }

    // =======================================================
    // SquareTracking
    // =======================================================

    case MOTION_COMMAND::SQUARE_TRACKING: {

      /*
       * SquareTracking.csv
       *
       * [0]  command
       * [1]  id
       * [2]  speed
       * [3]  targetX
       * [4]  kp
       * [5]  ki
       * [6]  kd
       * [7]  isStopMotorPower
       * [8]  roiX
       * [9]  roiY
       * [10] roiWidth
       * [11] roiHeight
       */

      if(motionParams.size() < 12) {
        Logger::error(
            "[MotionParser] SquareTrackingのパラメータ数が不足しています");

        return nullptr;
      }

      CameraServer::SquareDetectorRequest squareRequest {};

      squareRequest.roi.x =
          fromString<int32_t>(
              motionParams[8]);

      squareRequest.roi.y =
          fromString<int32_t>(
              motionParams[9]);

      squareRequest.roi.width =
          fromString<int32_t>(
              motionParams[10]);

      squareRequest.roi.height =
          fromString<int32_t>(
              motionParams[11]);

      const double targetSpeed =
          fromString<double>(
              motionParams[2]);

      const int targetX =
          fromString<int>(
              motionParams[3]);

      const Pid::PidGain squarePid = {
          fromString<double>(
              motionParams[4]),

          fromString<double>(
              motionParams[5]),

          fromString<double>(
              motionParams[6])
      };

      const bool isStopMotorPower =
          motionParams[7] == "true";

      Logger::printfLog(
          Logger::INFO,
          "[MotionParser] SquareTracking "
          "speed=%.2f "
          "targetX=%d "
          "PID=(%.7f,%.7f,%.7f) "
          "ROI=(%d,%d,%d,%d)",
          targetSpeed,
          targetX,
          squarePid.kp,
          squarePid.ki,
          squarePid.kd,
          squareRequest.roi.x,
          squareRequest.roi.y,
          squareRequest.roi.width,
          squareRequest.roi.height);

      return new CameraTracking(
          robot,
          move(condition),
          targetSpeed,
          targetX,
          squarePid,
          squareRequest,
          isStopMotorPower);
    }

    // =======================================================
    // 未定義
    // =======================================================

    default:

      Logger::printfLog(
          Logger::WARNING,
          "[MotionParser] Command %s は未実装です",
          motionParams[0].c_str());

      return nullptr;
  }
}

/**
 * @brief 文字列からMotionコマンドへ変換する
 */
MotionParser::MOTION_COMMAND
MotionParser::convertCommand(
    const string& str)
{
  static const unordered_map<
      string,
      MOTION_COMMAND>
      commandMap = {

          {
              "Straight",
              MOTION_COMMAND::STRAIGHT
          },

          {
              "QRTracking",
              MOTION_COMMAND::QR_TRACKING
          },

          {
              "SquareTracking",
              MOTION_COMMAND::SQUARE_TRACKING
          },
      };

  const auto it =
      commandMap.find(str);

  if(it != commandMap.end()) {
    return it->second;
  }

  return MOTION_COMMAND::NONE;
}

/**
 * @brief 文字列から継続条件へ変換する
 */
MotionParser::CONDITION_COMMAND
MotionParser::convertCondition(
    const string& str)
{
  static const unordered_map<
      string,
      CONDITION_COMMAND>
      conditionMap = {

          {
              "Distance",
              CONDITION_COMMAND::DISTANCE
          },
      };

  const auto it =
      conditionMap.find(str);

  if(it != conditionMap.end()) {
    return it->second;
  }

  return CONDITION_COMMAND::NONE;
}