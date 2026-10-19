
/**
 * @file   EtRobocon2026.cpp
 * @brief  ETラリー単独試走用
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

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
  // コース設定
  // ========================================

  // Rコース
  robot.setCourse(Course::Right);
  robot.setEdge(Edge::LeftEdge);

  // Lコース
  // robot.setCourse(Course::Left);
  // robot.setEdge(Edge::RightEdge);

  // ========================================
  // ゲート情報の仮登録
  // ========================================

  MapData& mapData = robot.getMapDataInstance();

  // 赤ゲート：Lコース座標 (7,9) ～ (9,9)
  mapData.setGate(GoalColor::RED, 25, 15, robot.getCourse());

  // 青ゲート：Lコース座標 (7,5) ～ (7,7)
  mapData.setGate(GoalColor::BLUE, 23, 24, robot.getCourse());

  // 黄ゲート：Lコース座標 (1,5) ～ (3,5)
  mapData.setGate(GoalColor::YELLOW, 53, 43, robot.getCourse());

  Logger::info("ET Rally: test gate data registered");

  // ========================================
  // ETラリー走行
  // ========================================

  Area etRallyArea = Area::ETRally;
  AreaMaster etRallyAreaMaster(robot, etRallyArea);
  etRallyAreaMaster.run();
}
