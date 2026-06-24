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
  request.requireLargestColorIndex = false;
  request.hsvRangeCount = 1;
  request.hsvRanges[0].lower = { 0, 0, 0 };
  request.hsvRanges[0].upper = { 180, 255, 30 };
  request.roi = { 320, 240, 1920, 1080 };

  CameraTracking cameraTracking(robot, std::make_unique<DistanceCondition>(robot, 8000.0), 100.0,
                                960, camPid, request);
  cameraTracking.run();

  // Logger::outputToFile();
}