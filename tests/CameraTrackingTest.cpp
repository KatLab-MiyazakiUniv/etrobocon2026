/**
 * @file CameraTrackingTest.cpp
 * @brief CameraTrackingクラスのテスト
 * @author sadomiya-sousi
 */

#include <gtest/gtest.h>
#include "CameraTracking.h"
#include "MockNetworkSystem.h"
#include "BaseContinuationCondition.h"
#include "Robot.h"
#include "ImageProcessingColor.h"

namespace etrobocon2026_test {

  class SimpleContinuationCondition : public BaseContinuationCondition {
   public:
    explicit SimpleContinuationCondition(Robot& robot) : BaseContinuationCondition(robot) {}
    void prepare() override {}
    bool shouldContinue() override { return false; }
  };

  class TestCameraTracking : public CameraTracking {
   public:
    using CameraTracking::CameraTracking;
    using CameraTracking::canStart;
  };

  // 目標速度が正の場合に canStart が true を返すか検証
  TEST(CameraTrackingTest, CanStartReturnsTrueForPositiveSpeed)
  {
    MockNetworkSystem mockNet;
    SocketClient mockClient(mockNet);
    Robot robot(mockClient);

    Pid::PidGain gain(0.1, 0.0, 0.0);
    CameraServer::ColorRegionDetectorRequest req;
    req.hsvRangeCount = 1;
    req.hsvRanges[0] = ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLACK);
    req.roi = { 0, 0, 640, 480 };

    TestCameraTracking tracking(robot, std::make_unique<SimpleContinuationCondition>(robot), 50.0,
                                320, gain, gain, gain, req);

    EXPECT_TRUE(tracking.canStart());
  }

  // 目標速度が0の場合に canStart が false を返すか検証
  TEST(CameraTrackingTest, CanStartReturnsFalseForZeroSpeed)
  {
    MockNetworkSystem mockNet;
    SocketClient mockClient(mockNet);
    Robot robot(mockClient);

    Pid::PidGain gain(0.1, 0.0, 0.0);
    CameraServer::ColorRegionDetectorRequest req;
    req.hsvRangeCount = 1;
    req.hsvRanges[0] = ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLACK);
    req.roi = { 0, 0, 640, 480 };

    TestCameraTracking tracking(robot, std::make_unique<SimpleContinuationCondition>(robot), 0.0,
                                320, gain, gain, gain, req);

    EXPECT_FALSE(tracking.canStart());
  }

  // コンストラクタでメンバ変数が正しく初期化されているかを検証
  TEST(CameraTrackingTest, ConstructorInitializesMembersCorrectly)
  {
    MockNetworkSystem mockNet;
    SocketClient mockClient(mockNet);
    Robot robot(mockClient);

    Pid::PidGain gain(0.1, 0.0, 0.0);
    CameraServer::ColorRegionDetectorRequest req;
    req.hsvRangeCount = 1;
    req.hsvRanges[0] = ImageProcessingColor::getHSVRangeFromColor(ImageProcessingColor::BLACK);
    req.roi = { 0, 0, 640, 480 };

    double speed = 75.5;
    int targetX = 350;

    TestCameraTracking tracking(robot, std::make_unique<SimpleContinuationCondition>(robot), speed,
                                targetX, gain, gain, gain, req);

    EXPECT_DOUBLE_EQ(tracking.getTargetSpeed(), speed);
    EXPECT_EQ(tracking.getTargetXCoordinate(), targetX + CAM_MAX_WIDTH / 2);

    const auto& retReq = tracking.getDetectionRequest();
    EXPECT_EQ(retReq.hsvRangeCount, req.hsvRangeCount);
    EXPECT_EQ(retReq.roi.x, req.roi.x);
    EXPECT_EQ(retReq.roi.y, req.roi.y);
    EXPECT_EQ(retReq.roi.width, req.roi.width);
    EXPECT_EQ(retReq.roi.height, req.roi.height);

    EXPECT_TRUE(tracking.getIsStopMotorPower());
  }

}  // namespace etrobocon2026_test
