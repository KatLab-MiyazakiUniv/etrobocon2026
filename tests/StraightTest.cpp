/**
 * @file StraightTest.cpp
 * @brief Straightクラスをテストする
 * @author migaku2645
 */

#include <gtest/gtest.h>
#include "Straight.h"
#include "Mileage.h"
#include "DistanceCondition.h"
#include <iostream>

namespace etrobocon2026_test {

  //目標速度が正の時、executeStep()がモータにpower値を入力するかテスト
  TEST(StraightTest, executeStep)
  {
    Robot robot;
    double targetSpeed = 1000.0;     // 目標速度
    double targetDistance = 10.0;  // 目標距離

    //executeStep()前のpower値をゲット
    double beforeRightPower = robot.getWheelMotorControllerInstance().getRightPower();
    double beforeLeftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    //executeStep()でモータにpower値をセット
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    straight.executeStep();

    //セットされたpower値をゲット
    double afterRightPower = robot.getWheelMotorControllerInstance().getRightPower();
    double afterLeftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    
    // 左右ともに新たなpower値がセットされているか確認
    EXPECT_NE(beforeRightPower,afterRightPower);
    EXPECT_NE(beforeLeftPower,afterLeftPower);

  }

  //目標速度が0の時、executeStep()がモータにpower値(0)を入力するかテスト
  TEST(StraightTest, executeStepTargetSpeedzero)
  {
    Robot robot;
    double targetSpeed = 0.0;     // 目標速度
    double targetDistance = 10.0;  // 目標距離


    //executeStep()でモータにpower値をセット
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    

    //executeStep(robot,targetSpeed = 0.0)前のpower値をゲット
    double beforeRightPower = robot.getWheelMotorControllerInstance().getRightPower();
    double beforeLeftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    straight.executeStep();

    //executeStep(robot,targetSpeed = 0.0)後のpower値をゲット
    double afterRightPower = robot.getWheelMotorControllerInstance().getRightPower();
    double afterLeftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    
    // 左右ともに0がセットされているか確認
    EXPECT_EQ(0.0,beforeRightPower);
    EXPECT_EQ(0.0,beforeLeftPower);
    EXPECT_EQ(0.0,afterRightPower);
    EXPECT_EQ(0.0,afterLeftPower);

  }

  //目標速度が負の時、executeStep()がモータにpower値を入力するかテスト
  TEST(StraightTest, executeStepTargetDistanceNegative)
  {
    Robot robot;
    double targetSpeed = -1000.0;     // 目標速度
    double targetDistance = 10.0;  // 目標距離

    //executeStep()前のpower値をゲット
    double beforeRightPower = robot.getWheelMotorControllerInstance().getRightPower();
    double beforeLeftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    //executeStep()でモータにpower値をセット
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    straight.executeStep();

    //セットされたpower値をゲット
    double afterRightPower = robot.getWheelMotorControllerInstance().getRightPower();
    double afterLeftPower = robot.getWheelMotorControllerInstance().getLeftPower();

    
    // 左右ともに新たなpower値がセットされているか確認
    EXPECT_NE(beforeRightPower,afterRightPower);
    EXPECT_NE(beforeLeftPower,afterLeftPower);

  }


  // 目標距離が正の時、run()で直進後、走行距離が目標距離だけ増加するかテスト（誤差あり）
  TEST(StraightTest, Run)
  {
    Robot robot;
    double targetSpeed = 1000.0;     // 目標速度
    double targetDistance = 10.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    straight.run();
    robot.getWheelMotorControllerInstance().stopBoth();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    //走行距離と目標距離との誤差
    double diviation = 0.5;

    // 走行距離が目標距離の誤差の範囲にあることテスト
    EXPECT_NEAR(targetDistance, endMileage - startMileage,diviation);
  }

  // 目標距離が0の時、runで直進後、走行距離が増加しないことをテスト
  TEST(StraightTest, RunZero)
  {
    Robot robot;
    double targetSpeed = 50.0;    // 目標速度
    double targetDistance = 0.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);
  

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    straight.run();
    robot.getWheelMotorControllerInstance().stopBoth();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);
  
    // 目標距離と走行距離が等しいことをテスト
    EXPECT_EQ(targetDistance, endMileage - startMileage);
  }

  // 目標距離が負の時、runで直進後、走行距離が増加しないことをテスト
  TEST(StraightTest, RunNegative)
  {
    Robot robot;
    double targetSpeed = 50.0;       // 目標速度
    double targetDistance = -100.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);
  

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    straight.run();
    robot.getWheelMotorControllerInstance().stopBoth();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);
    
    // 走行距離が0であることをテスト
    double expected = 0.0;
    EXPECT_EQ(expected, endMileage - startMileage);
  }

  // finishが正常に動作するかテスト
  TEST(StraightTest, finish)
  {
    Robot robot;
    double targetSpeed = 1000.0;     // 目標速度
    double targetDistance = 10.0;  // 目標距離

    // 直進前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double startMileage = Mileage::calculateMileage(rightCount, leftCount);

    // 直進動作を実行
    Straight straight(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                      targetSpeed);
    straight.run();
    straight.finish();

    // 直進後の走行距離を計算
    rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    leftCount = robot.getWheelMotorControllerInstance().getLeftCount();
    double endMileage = Mileage::calculateMileage(rightCount, leftCount);

    //走行距離と目標距離との誤差
    double diviation = 0.5;

    // 実際の走行距離が目標距離の誤差の範囲にあることテスト
    EXPECT_NEAR(targetDistance, endMileage - startMileage,diviation);
  }

}  // namespace etrobocon2026_test
