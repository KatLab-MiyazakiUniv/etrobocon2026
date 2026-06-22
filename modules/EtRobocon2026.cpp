


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
  request.requireLargestColorIndex = false;
  request.hsvRangeCount = 1;
  // くろ
  request.hsvRanges[0].lower = { 0, 0, 0 };
  request.hsvRanges[0].upper = { 180, 255, 30 };
  // 青　{ 100.0, 100.0, 100.0, 0.0 }, { 130.0, 255.0, 255.0, 0.0 }
  request.roi = { 320, 240, 1920, 1080 };

  // 継続条件>インスタンス化のための引数がおえてない
  CameraTracking cameraTracking(robot, std::make_unique<DistanceCondition>(robot, 5000.0), 600.0,
                                960, camPid, request);
  cameraTracking.run();

  // Logger::outputToFile();
}