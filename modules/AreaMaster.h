/**
 * @file   AreaMaster.h
 * @brief  エリアを攻略するクラス
 * @author nishijima515
 */

#ifndef AREA_MASTER_H
#define AREA_MASTER_H

#include <vector>
#include <array>
#include "Course.h"
#include "MotionParser.h"
#include "Robot.h"
#include "BaseMotion.h"

class AreaMaster {
 public:
  /**
   * コンストラクタ
   * @param robot Robotインスタンスの参照
   * @param area エリアの指定(Enum型のArea)
   */
  AreaMaster(Robot& _robot, Area _area);

  /**
   * @brief エリアを走行する
   */
  void run();

 private:
  Robot& robot;       // Robotインスタンスの参照
  Area area;          // エリアの指定(Enum型のArea)

  // 各エリアのコマンドファイルベースパス
  static const std::string basePath;

  // コマンドファイル名（各エリア名）
  static const std::array<std::string, 2> areaCommandNames;

  /**
   * @brief 動作リストを実行してメモリを解放する
   * @param motionList 実行する動作リスト
   */
  void executeMotions(std::vector<BaseMotion*>& motionList);
};

#endif