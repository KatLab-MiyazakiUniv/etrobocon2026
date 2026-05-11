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
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <stdexcept>
#include "BaseMotion.h"
#include "Area.h"

enum class COMMAND { EXAMPLE, NONE };

class MotionParser {
 public:
  /**
   * @brief datafiles/commands/Areaに配置した[Area名 + Left or
   * Right].csvからコマンド名とIDを抽出し、
   *        datafiles/commands/Motionsに配置したコマンド名に対応するcsvファイルから一致するIDの行を抽出し、
   *        datafiles/commands/Runに実行用のcsvファイルrun_[Area名 + Left or
   * Right].csvを作成する関数 (キャリブレーションで使う。)
   * @param robot ロボット本体の参照
   * @param area エリア
   * @param isLeftCourse 左コースか右コースか
   */
  static void createRunCSV(Robot& robot, Area area, bool isLeftCourse);

  /**
   * @brief
   * 指定したcsvファイルを読み込み、各コマンドのパラメータ値が関数の引数の型と一致しているか確認する関数
   *        (キャリブレーション中にrun[].csvファイルを作成した後に使用する)
   * @param commandFilePath 対象のcsvファイルパス
   * @return すべて一致していればtrue、不一致があればfalse
   */
  static bool checkType(std::string& commandFilePath);

  /**
   * @brief ファイルを解析して動作インスタンスのリストを生成する
   * @param robot ロボット本体の参照
   * @param commandFilePath ファイルパス
   * @return 動作インスタンスリスト
   */

  static std::vector<BaseMotion*> createMotionList(Robot& robot, std::string& commandFilePath);

 private:
  MotionParser();  // インスタンス化を禁止する

  static const std::array<std::string, 2> areaCommandNames;

  /**
   * @brief 文字列を列挙型COMMANDに変換する
   * @param str 文字列のコマンド
   * @return コマンド
   */
  static COMMAND convertCommand(const std::string& str);

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
