/**
 * @file   GatePositionParser.h
 * @brief  平文からゲート位置情報を解析するクラス
 * @author migaku2645
 */

#ifndef GATE_POSITION_PARSER_H
#define GATE_POSITION_PARSER_H

#include <string>
#include <sstream>
#include "Logger.h"
#include "Robot.h"

class GatePositionParser {
 public:
  /**
   * @brief コンストラクタ
   * @param _plaintext AES復号後の平文
   * @param _robot ゲート位置情報を設定するRobot
   */
  GatePositionParser(const std::string& _plaintext, Robot& _robot);

  /**
   * @brief デストラクタ
   */
  ~GatePositionParser();

  /**
   * @brief 平文を解析してゲート位置情報を設定する
   * @return 解析に成功した場合true
   */
  bool parse();

 private:
  std::string plaintext;
  Robot& robot;

  /**
   * @brief 赤色ゲートの位置情報を解析する
   * @return 解析に成功した場合true
   */
  bool parseRedPosition();

  /**
   * @brief 青色・黄色ゲートの位置情報を解析する
   * @return 解析に成功した場合true
   */
  bool parseBlueYellowPosition();

  /**
   * @brief XY,XY形式の位置情報を解析する
   * @param text 解析対象文字列
   * @param positions 位置情報を格納する配列
   * @return 解析に成功した場合true
   */
  bool parsePosition(const std::string& text, int positions[2]);

  /**
   * @brief 赤色ゲート形式か判定する
   * @return 赤色ゲート形式の場合true
   */
  bool isRedFormat() const;

  /**
   * @brief 青色・黄色ゲート形式か判定する
   * @return 青色・黄色ゲート形式の場合true
   */
  bool isBlueYellowFormat() const;
};

#endif