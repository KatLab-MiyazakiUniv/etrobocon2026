#include "SocketProtocol.h"
#include "Logger.h"
#include <gtest/gtest.h>

using namespace CameraServer;

namespace etrobocon2026_test {

  // SocketProtocolで定義した,構造体のサイズ(バイト)を表示するテスト
  TEST(SocketProtocol, sizeOfStructs)
  {
    Logger::info("=== 型サイズ一覧 (bytes) ===");
    Logger::printfLog(Logger::INFO, "Command: %zu", sizeof(Command));
    Logger::printfLog(Logger::INFO, "ScalarData: %zu", sizeof(ScalarData));
    Logger::printfLog(Logger::INFO, "RectData: %zu", sizeof(RectData));
    Logger::printfLog(Logger::INFO, "SizeData: %zu", sizeof(SizeData));
    Logger::printfLog(Logger::INFO, "PointData: %zu", sizeof(PointData));
    Logger::printfLog(Logger::INFO, "BoundingBoxDetectionResult: %zu",
                      sizeof(BoundingBoxDetectionResult));
    Logger::printfLog(Logger::INFO, "HSVRangeData: %zu", sizeof(HSVRangeData));
    Logger::printfLog(Logger::INFO, "ColorRegionDetectorRequest: %zu",
                      sizeof(ColorRegionDetectorRequest));
    Logger::printfLog(Logger::INFO, "ColorRegionDetectorResponse: %zu",
                      sizeof(ColorRegionDetectorResponse));
  }

}  // namespace etrobocon2026_test
