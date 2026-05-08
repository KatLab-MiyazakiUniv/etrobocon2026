/**
 * @file   AreaMaster.h
 * @brief  エリアを攻略するクラス
 * @author nishijima515
 */

#ifndef AREA_MASTER_H
#define AREA_MASTER_H

#include <vector>
#include <array>
#include "Area.h"
#include "MotionParser.h"
//Robot.hとBaseMotion.hはAreaMaster.hでincludeしないとエラーが出た。
#include "Robot.h"     
#include "BaseMotion.h"

class AreaMaster {
 public:
  /**
   * コンストラクタ
   * @param robot Robotインスタンスの参照
   * @param area エリアの指定(Enum型のArea)
   * @param isLeftCourse コースのLR判定(true:Lコース, false:Rコース)
   */
  AreaMaster(Robot& _robot, Area _area, bool _isLeftCourse);

  /**
   * @brief エリアを走行する
   */
  void run();

  /**
   * @brief エリア名を返す
   * @param area エリアの指定(Enum型のArea)
   * @return エリア名
   */
  // static std::string getAreaName(Area area);

 private:
  Robot& robot;          // Robotインスタンスの参照
  Area area;             // エリアの指定(Enum型のArea)
  bool isLeftCourse;     // コースのLR判定(true:Lコース, false:Rコース)

  // 各エリアのコマンドファイルベースパス
  static const std::string basePath;

  // コマンドファイル名（各エリア名）
  static const std::array<std::string, 2> areaCommandNames;

  /**
   * @brief 動作リストを実行してメモリを解放する
   * @param motionList 実行する動作リスト
   */
  void executeMotions(std::vector<BaseMotion*>& motionList);

//   /**
//    * @brief 条件分岐用ファイルパスを生成
//    * @param suffix ファイル名に付加するサフィックス（_Common, _FigJudgeSuccess, _FigJudgeFailure）
//    * @return 生成されたファイルパス
//    */
//   std::string getConditionalFilePath(const std::string& suffix);
};

#endif