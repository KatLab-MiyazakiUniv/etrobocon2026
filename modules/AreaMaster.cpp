/**
 * @file   AreaMaster.cpp
 * @brief  エリアを攻略するクラス
 * @author nishijima515
 */

#include "AreaMaster.h"
#include <fstream>
#include <iostream>

using namespace std;
const string AreaMaster::basePath
    = "etrobocon2026/datafiles/commands/Area/";  // エリアコマンドCSVファイルを置いているディレクトリのパス
const array<string, 5> AreaMaster::areaCommandNames
    = { "LineTrace", "RedBottleDelivery", "BlueBottleDelivery", "YellowBottleDelivery",
        "Area2" };  // エリア名の配列

AreaMaster::AreaMaster(Robot& _robot, Area _area) : robot(_robot), area(_area) {}

void AreaMaster::run()
{
  // ファイルから受け取る動作リスト
  vector<BaseMotion*> motionList;

  // コマンドファイルパスを作成する
  if(area == Area::BottleDelivery) {
    commandFilePath = basePath + areaCommandNames[static_cast<int>(area) + robot.getIndexOfLabel()]
                      + (robot.getCourse() == Course::Left ? "Left" : "Right") + ".csv";
  } else {
    commandFilePath = basePath + areaCommandNames[static_cast<int>(area)]
                      + (robot.getCourse() == Course::Left ? "Left" : "Right") + ".csv";
  }
  Logger::printfLog(Logger::INFO, "%s を開きました", commandFilePath.c_str());
  // 動作インスタンスのリストを生成する
  motionList = MotionParser::createMotionList(robot, commandFilePath);
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