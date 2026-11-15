/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"

void EtRobocon2026::start()
{
  CsvLogger::init();
  CsvLogger::writeHeader();

  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);

  robot.getCameraSocketClientInstance().connectToServer();

  int voltage = BatteryController::getVoltage();
  Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);

  // runlogのファイル名を作成
  std::tm jstTime = Utils::getLocalTime();
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), "runlog_%d-%d_%d-%d-%d_voltage%dmv.csv", jstTime.tm_mon,
                jstTime.tm_mday, jstTime.tm_hour, jstTime.tm_min, jstTime.tm_sec, voltage);
  std::string runlogFileName = buffer;
  CsvLogger::setFileName(runlogFileName);

  // Calibrator calibrator(robot, std::move(std::make_unique<RepeatCountCondition>(robot, 1)));
  // calibrator.run();
  // コース設定
  // Rコース
  // robot.setCourse(Course::Right);
  // robot.setEdge(Edge::LeftEdge);

  // Lコース
  robot.setCourse(Course::Left);
  robot.setEdge(Edge::RightEdge);

  // ETラリーのゲート情報を設定
  MapData& mapData = robot.getMapDataInstance();
  const Course course = robot.getCourse();

  // 赤ゲート：33 → 43
  bool redGate = mapData.setGate(GoalColor::RED, 25, 35, course);

  // 青ゲート：14 → 15
  bool blueGate = mapData.setGate(GoalColor::BLUE, 53, 54, course);

  // 黄ゲート：11 → 21
  bool yellowGate = mapData.setGate(GoalColor::YELLOW, 12, 22, course);

  if(!redGate || !blueGate || !yellowGate) {
    Logger::error("ET Rally: gate setting failed");
    CsvLogger::outputToFile();
    return;
  }

  Logger::info("ET Rally: gate setting completed");

  // ライントレース走行
  // Area lineTraceArea = Area::LineTrace;
  // AreaMaster lineTraceAreaMaster(robot, lineTraceArea);
  // lineTraceAreaMaster.run();

  // 調整のための一時的な走行
  // Area tmpArea = Area::Tmp;
  // AreaMaster tmpAreaMaster(robot, tmpArea);
  // tmpAreaMaster.run();
  // ボトルデリバリー走行
  // Area bottleDeliveryArea = Area::BottleDelivery;
  // AreaMaster bottleDeliveryAreaMaster(robot, bottleDeliveryArea);
  // bottleDeliveryAreaMaster.run();

  // CsvLogger::outputToFile();

  // // ET相撲走行
  // Area ETZumo = Area::ETZumo;
  // AreaMaster ETZumouAreaMaster(robot, ETZumo);
  // ETZumouAreaMaster.run();

  // ETラリー走行
  Area etRallyArea = Area::ETRally;
  AreaMaster etRallyAreaMaster(robot, etRallyArea);
  etRallyAreaMaster.run();

  CsvLogger::outputToFile();
}