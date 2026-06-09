/**
 * @file   EtRobocon2026.cpp
 * @brief  全体を制御するクラス
 * @author HaruArima08
 */

#include "EtRobocon2026.h"
#include "Robot.h"
#include "DistanceCondition.cpp"
#include "Pid.h"
#include "CameraTracking.h"

void EtRobocon2026::start()
{
  Logger::info("Hello KATLAB");

  RealNetworkSystem real;
  SocketClient client(real);
  Robot robot(client);
  robot.getCameraSocketClientInstance().connectToServer();
  Pid::PidGain camPid{ 0.002, 0.0005, 0.001 };
  Pid::PidGain rightPid{ 0.00535, 0.00115, 0.00 };
  Pid::PidGain leftPid{ 0.00578, 0.0008535, 0.00 };
  // DistanceCondition distanceCondition(robot, 100.0);
  CameraServer::ColorRegionDetectorRequest request;

  // 継続条件>インスタンス化のための引数がおえてない
  CameraTracking cameraTracking(robot, std::make_unique<DistanceCondition>(robot, 1500.0), 300.0,
                                960, camPid, rightPid, leftPid, request);
  cameraTracking.run();

  // cameraTracking.prepare();
  // if(cameraTracking.canStart()) {
  //   while(cameraTracking.continuationCondition.) {
  //     executeStep();
  //   }
  //   cameraTracking.finish();
  // } else {
  //   Logger::error("開始条件を満たしていません");
  // }
}
