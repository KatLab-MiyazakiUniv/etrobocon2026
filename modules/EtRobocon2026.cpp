
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
  //  CameraServer::ColorRegionDetectorRequest request;
  //  request.requireLargestColorIndex = true;
  //  request.hsvRangeCount = 2;
  // // くろ
  //  request.hsvRanges[0].lower ={ 0, 0, 0, 0 };
  //  request.hsvRanges[0].upper ={180, 255, 30, 0};
  // // 青　{ 100.0, 100.0, 100.0, 0.0 }, { 130.0, 255.0, 255.0, 0.0 }
  //  request.hsvRanges[1].lower ={ 100.0, 100.0, 100.0, 0.0 };
  //  request.hsvRanges[1].upper ={ 130.0, 255.0, 255.0, 0.0 };
  //  request.roi = { 320, 240, 1920, 1080 };

  //   // 継続条件>インスタンス化のための引数がおえてない
  //   CameraTracking cameraTracking(robot, std::make_unique<DistanceCondition>(robot, 1000.0),
  //   600.0,
  //                                 0, camPid, rightPid, leftPid, request);
  //   cameraTracking.run();

  //  CameraServer::ColorRegionDetectorRequest request2;
  //  request2.requireLargestColorIndex = false;
  //  request2.hsvRangeCount = 1;
  // // くろ
  //  request2.hsvRanges[0].lower ={ 0, 0, 0, 0 };
  //  request2.hsvRanges[0].upper ={180, 255, 30, 0};
  //  request2.roi = { 320, 240, 1920, 1080 };

  //   // 継続条件>インスタンス化のための引数がおえてない
  //   CameraTracking cameraTracking2(robot, std::make_unique<DistanceCondition>(robot, 1000.0),
  //   600.0,
  //                                 0, camPid, rightPid, leftPid, request2);
  //   cameraTracking2.run();

  // -------ボトルの色が取得できそうか------

  // インデックスの一つ目が赤
  CameraServer::ColorRegionDetectorRequest request3;
  request3.requireLargestColorIndex = true;
  request3.hsvRangeCount = 3;
  // 赤
  request3.hsvRanges[0].lower = { 0.0, 100.0, 100.0, 0.0 };
  request3.hsvRanges[0].upper = { 10.0, 255.0, 255.0, 0.0 };
  // 青
  request3.hsvRanges[1].lower = { 100.0, 100.0, 100.0, 0.0 };
  request3.hsvRanges[1].upper = { 130.0, 255.0, 255.0, 0.0 };
  // 黄
  request3.hsvRanges[2].lower = { 20.0, 100.0, 100.0, 0.0 };
  request3.hsvRanges[2].upper = { 35.0, 255.0, 255.0, 0.0 };

  request3.roi = { 320, 240, 1920, 1080 };

  CameraTracking cameraTracking3(robot, std::make_unique<DistanceCondition>(robot, 100.0), 600.0, 0,
                                 camPid, rightPid, leftPid, request3);
  cameraTracking3.run();

  // インデックスの2つ目が赤
  CameraServer::ColorRegionDetectorRequest request4;
  request4.requireLargestColorIndex = true;
  request4.hsvRangeCount = 3;
  // 青
  request4.hsvRanges[0].lower = { 100.0, 100.0, 100.0, 0.0 };
  request4.hsvRanges[0].upper = { 130.0, 255.0, 255.0, 0.0 };
  // 赤
  request4.hsvRanges[1].lower = { 0.0, 100.0, 100.0, 0.0 };
  request4.hsvRanges[1].upper = { 10.0, 255.0, 255.0, 0.0 };
  // 黄
  request4.hsvRanges[2].lower = { 20.0, 100.0, 100.0, 0.0 };
  request4.hsvRanges[2].upper = { 35.0, 255.0, 255.0, 0.0 };

  request4.roi = { 320, 240, 1920, 1080 };

  CameraTracking cameraTracking4(robot, std::make_unique<DistanceCondition>(robot, 100.0), 600.0, 0,
                                 camPid, rightPid, leftPid, request4);
  cameraTracking4.run();

  // インデックスの3つ目が赤
  CameraServer::ColorRegionDetectorRequest request5;
  request5.requireLargestColorIndex = true;
  request5.hsvRangeCount = 3;
  // 青
  request5.hsvRanges[0].lower = { 100.0, 100.0, 100.0, 0.0 };
  request5.hsvRanges[0].upper = { 130.0, 255.0, 255.0, 0.0 };
  // 黄
  request5.hsvRanges[1].lower = { 20.0, 100.0, 100.0, 0.0 };
  request5.hsvRanges[1].upper = { 35.0, 255.0, 255.0, 0.0 };
  // 赤
  request5.hsvRanges[2].lower = { 0.0, 100.0, 100.0, 0.0 };
  request5.hsvRanges[2].upper = { 10.0, 255.0, 255.0, 0.0 };

  request5.roi = { 320, 240, 1920, 1080 };

  CameraTracking cameraTracking5(robot, std::make_unique<DistanceCondition>(robot, 100.0), 600.0, 0,
                                 camPid, rightPid, leftPid, request5);
  cameraTracking5.run();

  // cameraTracking.prepare();
  // if(cameraTracking.canStart()) {
  //   while(cameraTracking.continuationCondition.) {
  //     executeStep();
  //   }
  //   cameraTracking.finish();
  // } else {
  //   Logger::error("開始条件を満たしていません");
  // }
  Logger::outputToFile();
}
