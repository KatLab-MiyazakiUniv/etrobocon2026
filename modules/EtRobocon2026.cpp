/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "AreaMaster.h"
#include "Calibrator.h"
#include "RealNetworkSystem.h"
#include "RepeatCountCondition.h"
#include "Robot.h"
#include "SocketClient.h"
#include "CsvLogger.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");
}

void EtRobocon2026::runLineTrace()
{
  CsvLogger::init();
  CsvLogger::writeHeader();
  // ライントレースではカメラ通信を行わないが、Robot が共有リソースとして
  // SocketClient を保持するため、ここで実機用の依存関係を構築する。
  RealNetworkSystem networkSystem;
  SocketClient cameraSocketClient(networkSystem);
  Robot robot(cameraSocketClient);

  // LineTraceLeft.csv と左エッジを使ってライントレース区間を走行する。
  robot.setCourse(Course::Left);
  robot.setEdge(Edge::RightEdge);

  // Area CSVを解析する前に黒・白の輝度を測定する。
  // MotionParserは、この値にLineTrace.csvのオフセットを加えてPID目標値を作る。
  Calibrator calibrator(robot, std::make_unique<RepeatCountCondition>(robot, 1));
  calibrator.measureAndSetTargetBrightness();
  calibrator.waitForStart();

  AreaMaster lineTraceArea(robot, Area::LineTrace);
  lineTraceArea.run();
  CsvLogger::outputToFile();
}
