/**
 * @file   MotionParser.h
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#ifndef MOTION_PARSER_H
#define MOTION_PARSER_H

constexpr char SEPARATOR = ',';

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "BaseMotion.h"
#include "DistanceCondition.h"
#include "Logger.h"
#include "RelativeAngleCondition.h"
#include "RepeatCountCondition.h"

class MotionParser {
 public:
  // 動作コマンド名
  enum class MOTION_COMMAND {
    STRAIGHT,
    QR_TRACKING,
    LINETRACE,
    RELATIVE_ROTATION,
    SNAPSHOT,
    NONE
  };

  // 条件コマンド名
  enum class CONDITION_COMMAND {
    DISTANCE,
    RELATIVE_ANGLE,
    REPEAT_COUNT,
    NONE
  };

  /**
   * @brief Area CSVファイルを解析して動作インスタンスのリストを生成する
   * @param robot ロボット本体
   * @param commandFilePath Area CSVファイルパス
   * @return 動作インスタンスリスト
   */
  static std::vector<BaseMotion*> createMotionList(
      Robot& robot,
      std::string& commandFilePath);

 private:
  MotionParser();

  /**
   * @brief CSVファイルからIDに一致する行を取得する
   */
  static std::vector<std::string> extractParamsFromID(
      const std::string& filePath,
      const std::string& id);

  /**
   * @brief 条件インスタンスを生成する
   */
  static std::unique_ptr<BaseContinuationCondition> createConditionInstance(
      Robot& robot,
      const std::vector<std::string>& params);

  /**
   * @brief 動作インスタンスを生成する
   */
  static BaseMotion* createMotionInstance(
      Robot& robot,
      const std::vector<std::string>& motionParams,
      std::unique_ptr<BaseContinuationCondition> condition);

  /**
   * @brief 文字列をMOTION_COMMANDへ変換する
   */
  static MOTION_COMMAND convertCommand(
      const std::string& str);

  /**
   * @brief 文字列をCONDITION_COMMANDへ変換する
   */
  static CONDITION_COMMAND convertCondition(
      const std::string& str);
};

#endif