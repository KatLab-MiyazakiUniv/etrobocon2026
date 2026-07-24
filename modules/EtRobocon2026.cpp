// /**
//  * @file   EtRobocon2026.cpp
//  * @brief  全体を制御するクラス
//  * @author HaruArima08
//  */

// #include "EtRobocon2026.h"
// #include "AreaMaster.h"
// #include "Robot.h"
// #include "Course.h"
// #include "CsvLogger.h"
// #include "RealNetworkSystem.h"
// #include "BatteryController.h"

// void EtRobocon2026::start()
// {
//   CsvLogger::init();
//   CsvLogger::writeHeader();
//   Logger::info("Hello KATLAB");
//   RealNetworkSystem real;
//   SocketClient client(real);
//   Robot robot(client);
//   int voltage = BatteryController::getVoltage();
//   Logger::printfLog(Logger::INFO, "バッテリー電圧: %d mV", voltage);
//   robot.getCameraSocketClientInstance().connectToServer();
//   robot.setCourse(Course::Left);
//   // robot.setEdge(Edge::LeftEdge);
//   robot.setEdge(Edge::RightEdge);
//   Area lineTraceArea = Area::LineTrace;

//   AreaMaster lineTraceAreaMaster(robot, Area::LineTrace);

//   lineTraceAreaMaster.run();

//   if(robot.getIndexOfLabel() == 2) {
//     Area yellowArea = Area::Yellow;
//     AreaMaster carryAreaMaster(robot, Area::Yellow);
//     carryAreaMaster.run();
//   } else if(robot.getIndexOfLabel() == 1) {
//     Area blueArea = Area::Blue;
//     AreaMaster carryAreaMaster(robot, Area::Blue);
//     carryAreaMaster.run();

//   } else {
//     Area redArea = Area::Red;
//     AreaMaster carryAreaMaster(robot, Area::Red);
//     carryAreaMaster.run();
//   }

//   CsvLogger::outputToFile();
// }

/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "DistanceCondition.h"
#include "Pid.h"
#include "CameraTracking.h"
#include <vector>

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  robot.getCameraSocketClientInstance().connectToServer();
  Pid::PidGain camPid{ 0.0005, 0.0002, 0.0002 };
  Pid::PidGain rightPid{ 0.00535, 0.00115, 0.00 };
  Pid::PidGain leftPid{ 0.00578, 0.0008535, 0.00 };
  CameraServer::ColorRegionDetectorRequest request;
  request.requireLargestColorIndex = true;
  request.hsvRangeCount = 1;
  request.hsvRanges[0].lower = { 0, 0, 0 };
  request.hsvRanges[0].upper = { 180, 255, 30 };
  // request.hsvRanges[0] ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::RED);
  // request.hsvRanges[1] ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLUE);
  // request.hsvRanges[2] ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::YELLOW);
  request.roi = { 320, 240, 1920, 1080 };

  CameraTracking cameraTracking(robot, std::make_unique<DistanceCondition>(robot, 80000.0), 0.0,
                                960, camPid, request);
  cameraTracking.run();

  // Logger::outputToFile();
}