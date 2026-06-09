
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
#include  <vector>

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


  // /**
  //  * @brief カメラサーバーに色領域検出を要求する際のリクエスト構造体
  //  */
  // struct ColorRegionDetectorRequest {
  //   Command command = Command::COLOR_REGION_DETECTION;  // 色領域検出コマンド
  //   bool requireLargestColorIndex = false;   // 最も大きい色領域のインデックスを返すかどうか
  //   uint32_t hsvRangeCount = 0;              // hsvRangesの有効な要素数
  //   HSVRangeData hsvRanges[MAX_HSV_RANGES];  // HSVの範囲の配列
  //   RectData roi;                            // 検出対象の領域
  //   SizeData resolution;                     // 解像度
  // };
 CameraServer::ColorRegionDetectorRequest request;
 request.requireLargestColorIndex = true;
 request.hsvRangeCount = 1;
 request.hsvRanges[0].lower ={ 0, 0, 0, 0 };
 request.hsvRanges[0].upper ={180, 255, 30, 0};
 request.roi = { 0, 0, 1920, 1080 };
  request.resolution = { 1920, 1080 };
  // {CameraServer::Command::COLOR_REGION_DETECTION, true, 1, hsv, roi, resolution };

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
