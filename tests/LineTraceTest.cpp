/**
 * @file   LineTraceTest.cpp
 * @brief  LineTraceクラスをテストする
 * @author okuyama0528
 */

#include <gtest/gtest.h>
#include "LineTrace.h"
#include "DistanceCondition.h"
#include "Mileage.h"

namespace etrobocon2026_test {

  class LineTraceTest : public ::testing::Test {
   protected:
    Pid::PidGain rightPid = { 0.00535, 0.00115, 0.000 };
    Pid::PidGain leftPid = { 0.00578, 0.0008535, 0.000 };
    Pid::PidGain tracePid = { 0.0055, 0.0009, 0.0 };
  };

  TEST_F(LineTraceTest, Run)
  {
    Robot robot;

    double targetSpeed = 1000.0;
    double targetDistance = 10.0;
    int targetBrightness = 50;

    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, rightPid, leftPid, tracePid);

    lineTrace.run();

    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    EXPECT_NEAR(targetDistance, endMileage - startMileage, 0.5);
  }
  TEST_F(LineTraceTest, RunZeroSpeed)
  {
    Robot robot;

    double targetSpeed = 0.0;
    double targetDistance = 0.0;  // ★ここ重要（最初から終了条件満たす）
    int targetBrightness = 50;

    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, rightPid, leftPid, tracePid);

    lineTrace.run();

    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    EXPECT_EQ(0.0, endMileage - startMileage);
  }

  TEST_F(LineTraceTest, RunZeroDistance)
  {
    Robot robot;

    double targetSpeed = 100.0;
    double targetDistance = 0.0;
    int targetBrightness = 50;

    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, rightPid, leftPid, tracePid);

    lineTrace.run();

    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    EXPECT_EQ(0.0, endMileage - startMileage);
  }

  TEST_F(LineTraceTest, RunNegativeDistance)
  {
    Robot robot;

    double targetSpeed = 100.0;
    double targetDistance = -100.0;
    int targetBrightness = 50;

    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, rightPid, leftPid, tracePid);

    lineTrace.run();

    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    EXPECT_EQ(0.0, endMileage - startMileage);
  }

}  // namespace etrobocon2026_test