/**
 * @file   MotionParser.h
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#ifndef MOTION_PARSER_H
#define MOTION_PARSER_H

constexpr char SEPARATOR = ',';  // csvファイル内の区切り文字として、カンマを定義

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include "Logger.h"

#include "DistanceCondition.h"
#include "RelativeAngleCondition.h"
#include "AbsoluteAngleCondition.h"

#include "BaseMotion.h"
#include "RelativeRotation.h"
#include "AbsoluteRotation.h"
#include "Straight.h"

class MotionParser {
 public:
  // 動作コマンド名を持つ列挙型クラス
  enum class MOTION_COMMAND { STRAIGHT, ABSOLUTEROTATION, RELATIVEROTATION, NONE };
  // 条件コマンド名を持つ列挙型クラス
  enum class CONDITION_COMMAND { DISTANCE, ABSOLUTEANGLE, RELATIVEANGLE, NONE };

  /**
   * @brief Area CSVファイルを解析して動作インスタンスのリストを生成する
   * @param robot ロボット本体の参照
   * @param commandFilePath Area CSVファイルパス
   * @return 動作インスタンスリスト
   */
  static std::vector<BaseMotion*> createMotionList(Robot& robot, std::string& commandFilePath);

 private:
  MotionParser();  // インスタンス化を禁止する

  /**
   * @brief CSVファイルからIDに一致する行のパラメータを取得する
   * @param filePath CSVファイルパス
   * @param id 検索するID
   * @return パラメータリスト（見つからない場合は空のvector）
   */
  static std::vector<std::string> extractParamsFromID(const std::string& filePath,
                                                      const std::string& id);

  /**
   * @brief パラメータから条件インスタンスを生成する
   * @param robot ロボット本体の参照
   * @param params 条件CSVから取得したパラメータリスト
   * @return 条件インスタンス（未定義の場合は nullptr）
   */
  static std::unique_ptr<BaseContinuationCondition> createConditionInstance(
      Robot& robot, const std::vector<std::string>& params);

  /**
   * @brief パラメータから動作インスタンスを生成する
   * @param robot ロボット本体の参照
   * @param motionParams 動作CSVから取得したパラメータリスト
   * @param condition 注入する条件インスタンス
   * @return 動作インスタンス（未定義の場合は nullptr）
   */
  static BaseMotion* createMotionInstance(Robot& robot,
                                          const std::vector<std::string>& motionParams,
                                          std::unique_ptr<BaseContinuationCondition> condition);

  /**
   * @brief 文字列を列挙型MOTION_COMMANDに変換する
   * @param str 文字列のコマンド
   * @return コマンド
   */
  static MOTION_COMMAND convertCommand(const std::string& str);

  /**
   * @brief 文字列を列挙型CONDITION_COMMANDに変換する
   * @param str 文字列の条件コマンド
   * @return 条件コマンド
   */
  static CONDITION_COMMAND convertCondition(const std::string& str);

  // /**
  //  * @brief 文字列をbool型に変換する
  //  * @param command 文字列のコマンド
  //  * @param stringParameter 文字列のパラメータ
  //  * @return bool値
  //  */
  // static bool convertBool(const std::string& command, const std::string& stringParameter);

  // /**
  //  * @brief 回頭方法の文字列をbool型に変換する（convertBoolは方向判定で使用済みのため専用関数化）
  //  * @param stringParameter 文字列のパラメータ ("relative" or "absolute")
  //  * @return false: 相対角度回頭, true: 絶対角度回頭
  //  */
  // static bool convertRotationModeToBool(const std::string& stringParameter);
};

#endif
