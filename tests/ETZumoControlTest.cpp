#include <gtest/gtest.h>
#include "AbsoluteRotation.h"
#include "RelativeRotation.h"
#include "ETZumoExitCondition.h"
#include "ETZumoFinish.h"
#include "ProjectedDistanceCondition.h"
#include "SensorColorCondition.h"
#include "RepeatCountCondition.h"
#include "MockNetworkSystem.h"

namespace etrobocon2026_test {
  class AbsoluteRotationProbe : public AbsoluteRotation {
   public:
    using AbsoluteRotation::executeStep;
    using AbsoluteRotation::prepare;
    using AbsoluteRotation::AbsoluteRotation;
    void target(double angle) { targetAngle = angle; }
  };

  TEST(ETZumoControlTest, AbsoluteRotationOvercomesSmallPidOutputInBothDirections)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    AbsoluteRotationProbe rotation(robot, std::make_unique<RepeatCountCondition>(robot, 1),
                           { 0.001, 0, 0 }, 0);
    for(double angle : { 5.0, -5.0 }) {
      robot.getIMUControllerInstance().resetAzimuth();
      rotation.prepare();
      rotation.target(angle);
      rotation.executeStep();
      EXPECT_EQ(angle > 0 ? -18 : 18, robot.getWheelMotorControllerInstance().getRightPower());
      EXPECT_EQ(angle > 0 ? 18 : -18, robot.getWheelMotorControllerInstance().getLeftPower());
    }
    robot.getIMUControllerInstance().resetAzimuth();
    rotation.target(0);
    rotation.executeStep();
    EXPECT_EQ(0, robot.getWheelMotorControllerInstance().getRightPower());
  }

  class RelativeRotationProbe : public RelativeRotation {
   public:
    using RelativeRotation::executeStep;
    using RelativeRotation::prepare;
    using RelativeRotation::RelativeRotation;
  };

  TEST(ETZumoControlTest, RelativeRotationKeepsSmallPidOutput)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    for(double angle : { 5.0, -5.0 }) {
      robot.getIMUControllerInstance().resetAzimuth();
      RelativeRotationProbe rotation(robot, std::make_unique<RepeatCountCondition>(robot, 1),
                                     { 1.0, 0, 0 }, angle);
      rotation.prepare();
      rotation.executeStep();
      EXPECT_EQ(-angle, robot.getWheelMotorControllerInstance().getRightPower());
      EXPECT_EQ(angle, robot.getWheelMotorControllerInstance().getLeftPower());
    }
  }

  TEST(ETZumoControlTest, RequiresDistanceThenConsecutiveBlackAndResetsCount)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    auto mileage = std::make_shared<ProjectedMileage>();
    mileage->reset(0, 0);
    ETZumoExitCondition condition(
        robot, mileage, 700,
        std::make_unique<SensorColorCondition>(robot, ColorSensorController::COLOR::BLACK), true);
    condition.prepare();
    spikeapi::ColorSensor::setHSV({ 240, 80, 10 });
    for(int i = 0; i < 5; ++i) EXPECT_TRUE(condition.shouldContinue());
    // エンコーダ0のまま、起点を-700にして基準方向の距離700を作る。
    robot.getIMUControllerInstance().resetAzimuth();
    mileage->reset(-700, 0);
    spikeapi::ColorSensor::setHSV({ 0, 0, 60 });
    EXPECT_TRUE(condition.shouldContinue());
    spikeapi::ColorSensor::setHSV({ 240, 80, 10 });
    EXPECT_TRUE(condition.shouldContinue());
    EXPECT_TRUE(condition.shouldContinue());
    EXPECT_FALSE(condition.shouldContinue());
    condition.prepare();
    EXPECT_TRUE(condition.shouldContinue());
  }

  class RecordingMotion : public BaseMotion {
   public:
    RecordingMotion(Robot& robot, std::vector<int>& calls, int id)
      : BaseMotion(robot, std::make_unique<RepeatCountCondition>(robot, 1)), calls(calls), id(id)
    {
    }

   protected:
    void executeStep() override { calls.push_back(id); }

   private:
    std::vector<int>& calls;
    int id;
  };

  TEST(ETZumoControlTest, CompositeRunsChildrenInOrderAndStops)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    std::vector<int> calls;
    std::vector<std::unique_ptr<BaseMotion>> children;
    for(int i = 0; i < 3; ++i)
      children.push_back(std::make_unique<RecordingMotion>(robot, calls, i));
    ETZumoFinish motion(robot, std::make_unique<RepeatCountCondition>(robot, 1),
                        std::move(children));
    motion.run();
    EXPECT_EQ((std::vector<int>{ 0, 1, 2 }), calls);
    EXPECT_EQ(0, robot.getWheelMotorControllerInstance().getRightPower());
  }

  TEST(ETZumoControlTest, VerticalTargetUsesSharedSignedCoordinate)
  {
    MockNetworkSystem network;
    SocketClient client(network);
    Robot robot(client);
    auto mileage = std::make_shared<ProjectedMileage>();
    mileage->reset(701, -90);
    mileage->update(0, -90);  // 後退なのでY=+701
    ProjectedDistanceCondition positive(robot, mileage, ProjectedDistanceCondition::Axis::VERTICAL,
                                        700);
    positive.prepare();
    EXPECT_FALSE(positive.shouldContinue());
    ProjectedDistanceCondition negative(robot, mileage, ProjectedDistanceCondition::Axis::VERTICAL,
                                        -700);
    negative.prepare();
    EXPECT_TRUE(negative.shouldContinue());
    mileage->reset(-701, -90);
    mileage->update(0, -90);  // 前進なのでY=-701
    EXPECT_FALSE(negative.shouldContinue());
    EXPECT_NEAR(mileage->getVerticalDistance(), -701, 1e-4);
  }
}  // namespace etrobocon2026_test
