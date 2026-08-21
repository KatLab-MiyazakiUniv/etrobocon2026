#include "EtRobocon2026.h"
#include "AreaMaster.h"
#include "Robot.h"
#include "Course.h"
#include "CsvLogger.h"
#include "RealNetworkSystem.h"
#include "BatteryController.h"

void EtRobocon2026::start()
{
  CsvLogger::init();
  CsvLogger::writeHeader();
  Logger::info("Hello KATLAB");
  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  int voltage = BatteryController::getVoltage();
  Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);
  robot.getCameraSocketClientInstance().connectToServer();
  robot.setCourse(Course::Left);
  // robot.setEdge(Edge::LeftEdge);
  robot.setEdge(Edge::RightEdge);
  // // LineTrace走行
  // Area lineTraceArea = Area::LineTrace;
  // AreaMaster lineTraceAreaMaster(robot, lineTraceArea);
  // lineTraceAreaMaster.run();

//カメラトラッキング走行
  Area CameraTracking = Area::CameraTracking;
  AreaMaster CameraTrackingAreaMaster(robot, CameraTracking);
  CameraTrackingAreaMaster.run();

  // // // BotlleDelivery走行
  // Area bottleDeliveryArea = Area::BottleDelivery;
  // AreaMaster bottleDeliveryAreaMaster(robot, bottleDeliveryArea);
  // bottleDeliveryAreaMaster.run();

  // QRTracking走行
  // Area QRTrackingArea = Area::QRTracking;
  // AreaMaster QRTrackingAreaMaster(robot, QRTrackingArea);
  // QRTrackingAreaMaster.run();

  CsvLogger::outputToFile();
}