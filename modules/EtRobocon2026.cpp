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
#include "ClockUtil.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");
    RealNetworkSystem networkSystem;
  SocketClient cameraSocketClient(networkSystem);
  Robot robot(cameraSocketClient);
  
  robot
      .getCameraSocketClientInstance()
      .connectToServer();

        // HSV値を取得
//   ColorSensorController::HSV hsv1;

//   while(1){
//   robot.getColorSensorControllerInstance().getRawHSV(hsv1, true);
//   ColorSensorController::COLOR CurrentColor
//       = robot.getColorSensorControllerInstance().convertHsvToColor(hsv1);

// Logger::printfLog(
//   Logger::DEBUG,
//   "HSV: H=%u, S=%u, V=%u, color = %d",
//   static_cast<unsigned int>(hsv1.h),
//   static_cast<unsigned int>(hsv1.s),
//   static_cast<unsigned int>(hsv1.v),
//   CurrentColor
// );
// ClockUtil::wait(10);
//   };
  // LineTraceLeft.csv と左エッジを使ってライントレース区間を走行する。
  robot.setCourse(Course::Left);
  robot.setEdge(Edge::LeftEdge);


  AreaMaster lineTraceArea(robot, Area::LineTrace);
  lineTraceArea.run();
}
