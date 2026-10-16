/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

void EtRobocon2026::start()
{
  // CsvLogger::init();
  // CsvLogger::writeHeader();

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
  // ライントレース走行
  // ========================================

  Area lineTraceArea = Area::LineTrace;
  AreaMaster lineTraceAreaMaster(robot, lineTraceArea);
  lineTraceAreaMaster.run();

  // ========================================
  // ボトルデリバリー走行
  // ========================================

  Area bottleDeliveryArea = Area::BottleDelivery;
  AreaMaster bottleDeliveryAreaMaster(robot, bottleDeliveryArea);
  bottleDeliveryAreaMaster.run();

  // CsvLogger::outputToFile();

    // ========================================
  // ET相撲走行
  // ========================================

  Area ETZumo = Area::ETZumo;
  AreaMaster ETZumouAreaMaster(robot, ETZumo);
  ETZumouAreaMaster.run();

  // ========================================
  // ETラリー走行
  // ========================================

  Area etRallyArea = Area::ETRally;
  AreaMaster etRallyAreaMaster(robot, etRallyArea);
  etRallyAreaMaster.run();
}