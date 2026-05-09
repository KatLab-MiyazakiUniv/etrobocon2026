/**
 * @file   AreaMaster.cpp
 * @brief  エリアを攻略するクラス
 * @author nishijima515
 */

#include "AreaMaster.h"
#include <fstream>
#include <iostream>

using namespace std;
const string AreaMaster::basePath = "etrobocon2026/datafiles/commands/Run";
const array<string, 2> AreaMaster::areaCommandNames = { "LineTrace", "Area2" };

AreaMaster::AreaMaster(Robot& _robot, Area _area, bool _isLeftCourse)
  : robot(_robot), area(_area), isLeftCourse(_isLeftCourse)
{
}

void AreaMaster::run()
{
  // ファイルから受け取る動作リスト
  vector<BaseMotion*> motionList;

  // コマンドファイルパスを作成する
  string commandFilePath = basePath + "run_" + areaCommandNames[static_cast<int>(area)]
                           + (isLeftCourse ? "Left" : "Right") + ".csv";

  // 動作インスタンスのリストを生成する
  motionList = MotionParser::createMotionList(robot, commandFilePath);

  // 各動作を実行し、動作し終えたらメモリを開放する
  executeMotions(motionList);
}



void AreaMaster::executeMotions(vector<BaseMotion*>& motionList)
{
  for(auto motion = motionList.begin(); motion != motionList.end();) {
    (*motion)->run();
    delete *motion;                     // メモリを解放
    motion = motionList.erase(motion);  // リストから削除
  }
}

// string AreaMaster::getConditionalFilePath(const string& suffix)
// {
//   return basePath + areaCommandNames[static_cast<int>(area)] + (isLeftCourse ? "Left" : "Right")
//          + suffix + ".csv";
// }