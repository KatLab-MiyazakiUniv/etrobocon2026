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
#include "BaseMotion.h"
#include "Logger.h"

// 動作コマンド名を持つ列挙型クラス
enum class COMMAND { EXAMPLE, STRAIGHT, NONE };

// 条件コマンド名を持つ列挙型クラス
enum class CONDITION { DISTANCE, NONE };

class MotionParser {
 public:
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
   * @brief Conditions CSVから条件インスタンスを生成する
   * @param robot ロボット本体の参照
   * @param conditionName 条件コマンド名（Conditions/<conditionName>.csv を開く）
   * @param conditionId 取得する条件のID
   * @return 条件インスタンス（失敗時は nullptr）
   */
  static std::unique_ptr<BaseContinuationCondition> createConditionList(
      Robot& robot, const std::string& conditionName, const std::string& conditionId);

  /**
   * @brief 文字列を列挙型COMMANDに変換する
   * @param str 文字列のコマンド
   * @return コマンド
   */
  static COMMAND convertCommand(const std::string& str);

  /**
   * @brief 文字列を列挙型CONDITIONに変換する
   * @param str 文字列の条件コマンド
   * @return 条件コマンド
   */
  static CONDITION convertCondition(const std::string& str);

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
