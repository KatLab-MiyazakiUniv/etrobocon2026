/**

* @file   EtRobocon2026.cpp

* @brief  ETラリー単独試走用（ゲート情報を仮登録）

* @author HaruArima08

*/

#include "EtRobocon2026.h"

#include "CourseMirror.h"

#include "MapData.h"

#include "RouteTypes.h"

void EtRobocon2026::start()

{

Logger::info("Hello KATLAB");

RealNetworkSystem real;

SocketClient client(real);

Robot robot(client);

robot.getCameraSocketClientInstance().connectToServer();

int voltage = BatteryController::getVoltage();

Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);

// ========================================

// 試走用コース設定

// ========================================

robot.setCourse(Course::Right);

robot.setEdge(Edge::LeftEdge);

// Lコースで試走するときは上の2行をコメントアウトして使用

// robot.setCourse(Course::Left);

// robot.setEdge(Edge::RightEdge);

// ========================================

// 仮のゲート情報（Lコース基準）

// ========================================

MapData& mapData = robot.getMapDataInstance();

// Lコース基準のゲート座標を、Rコースなら左右反転

auto toCurrentCourse = &robot -> Point {

if(robot.getCourse() == Course::Right) {

return CourseMirror::mirrorPoint(point);

}

return point;

};

// 赤：横向きゲート

mapData.setGatePoint(GoalColor::RED,

toCurrentCourse({ 7, 9 }),

toCurrentCourse({ 9, 9 }));

// 青：縦向きゲート

mapData.setGatePoint(GoalColor::BLUE,

toCurrentCourse({ 7, 5 }),

toCurrentCourse({ 7, 7 }));

// 黄：横向きゲート

mapData.setGatePoint(GoalColor::YELLOW,

toCurrentCourse({ 1, 5 }),

toCurrentCourse({ 3, 5 }));

Logger::info("ET Rally: test gate data registered");

// ========================================

// ETラリーだけをMotionParser経由で実行

// ========================================

Area etRallyArea = Area::ETRally;

AreaMaster etRallyAreaMaster(robot, etRallyArea);

etRallyAreaMaster.run();

}