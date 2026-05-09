/**
 * @file   MotionParser.h
 * @brief  動作コマンドファイルを解析するクラス
 * @author nishijima515
 */

#ifndef MOTION_PARSER_H
#define MOTION_PARSER_H

constexpr char SEPARATOR = ',';  //csvファイル内の区切り文字として、カンマを定義

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include "BaseMotion.h"
#include "Area.h"

enum class COMMAND {
  NONE
};

class MotionParser {
 public:

// Areaからコマンド名とIDを抽出し、Motionsから対応するコマンドファイルのなかのIDの行を抽出し、走行中のみrun_[Area名 + Left or Right].csvを作成する関数
// キャリブレーションで使う。
// areaを引数で受け取り、areaごとのrun_[Area名 + Left or Right].csvを作成する関数
static void createRunCSV(Robot& robot, Area area, bool isLeftCourse);


//型チェック関数(publicに置く。キャリブレーション中にrun[].csvファイルを作成した後に、使う。)
// static bool checkType(std::string& commandFilePath);

//csvファイル内の　「#」以降はコメントとして扱う


//Areaの[Area名 + Left or Right].csvからコマンド名とIDを抽出(params[0]にコマンド名、params[1]にID)
//抽出したコマンド名に対応するdatafiles/commands/Motions内にある[コマンド名].csvファイルを読み込む
//[コマンド名].csvファイルのなかから対応するIDの行を読み込む
//読み込んだ行のパラメータをparams[2]以降に格納する。(ID,コマンドは飛ばす or params[0], params[1]に再格納)(カンマ区切り)
//キャリブレーション段階で、run + [Area名 + Left or Right].csvを作成。(command/Runに作成)


//キャリブレーション段階で、run + [Area名 + Left or Right].csvを作成。
//あとは去年のcreateMotionsをそのままAreaMasterで使う。
  /**
   * @brief ファイルを解析して動作インスタンスのリストを生成する
   * @param robot ロボット本体の参照
   * @param commandFilePath ファイルパス
   * @return 動作インスタンスリスト
   */

  static std::vector<Motion*> createMotionList(Robot& robot, std::string& commandFilePath);

 private:
  MotionParser();  // インスタンス化を禁止する

  static const std::array<std::string, 2> areaCommandNames;

  /**
   * @brief 文字列を列挙型COMMANDに変換する
   * @param str 文字列のコマンド
   * @return コマンド
   */
  // static COMMAND convertCommand(const std::string& str);

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
