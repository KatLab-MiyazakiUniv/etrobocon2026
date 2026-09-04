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
    // テスト用のPidゲイン
    Pid::PidGain tracePid = { 0.0055, 0.0009, 0.0001 };
    // テスト用のデッドバンド・マックスアウトの割合
    double deadbandRate = 0.05;
    double maxoutRate = 0.9;
  };

  // 目標距離が正の時、run()でライントレース後、走行距離が目標距離だけ増加するかテスト（誤差あり）
  TEST_F(LineTraceTest, Run)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    double targetSpeed = 1000.0;   // 目標速度
    double targetDistance = 10.0;  // 目標距離
    int targetBrightness = 50;     // 目標とする明るさの値(%)

    // ライントレース前の走行距離を計算
    int32_t rightCount = robot.getWheelMotorControllerInstance().getRightCount();
    int32_t leftCount = robot.getWheelMotorControllerInstance().getLeftCount();

    // 走行体全体の累計走行距離を計算
    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // ライントレース動作を実行
    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, tracePid, deadbandRate, maxoutRate);
    lineTrace.run();

    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // 走行距離と目標距離との誤差の範囲にあることテスト
    EXPECT_NEAR(targetDistance, endMileage - startMileage, 0.5);
  }

  // 目標速度が0の時、runでライントレース後、走行距離が増加しないことをテスト
  TEST_F(LineTraceTest, RunZeroSpeed)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    double targetSpeed = 0.0;     // 目標速度
    double targetDistance = 0.0;  // 目標距離
    int targetBrightness = 50;    // 目標とする明るさの値(%)

    // ライントレース前の走行距離を計算
    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // ライントレース動作を実行
    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, tracePid, deadbandRate, maxoutRate);
    lineTrace.run();

    // ライントレース後の走行距離を計算
    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // 目標距離と走行距離が等しいことをテスト
    EXPECT_EQ(0.0, endMileage - startMileage);
  }

  // 目標距離が0の時、runでライントレース後、走行距離が増加しないことをテスト
  TEST_F(LineTraceTest, RunZeroDistance)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    double targetSpeed = 100.0;   // 目標速度
    double targetDistance = 0.0;  // 目標距離
    int targetBrightness = 50;    // 目標とする明るさの値(%)

    // ライントレース前の走行距離を計算
    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // ライントレース動作を実行
    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, tracePid, deadbandRate, maxoutRate);
    lineTrace.run();

    // ライントレース後の走行距離を計算
    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // 目標距離と走行距離が等しいことをテスト
    EXPECT_EQ(0.0, endMileage - startMileage);
  }

  // 目標距離が負の時、runでライントレース後、走行距離が増加しないことをテスト
  TEST_F(LineTraceTest, RunNegativeDistance)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);       // Robotクラスのインスタンスを作成
    double targetSpeed = 100.0;      // 目標速度
    double targetDistance = -100.0;  // 目標距離
    int targetBrightness = 50;       // 目標とする明るさの値(%)

    // ライントレース前の走行距離を計算
    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // ライントレース動作を実行
    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, tracePid, deadbandRate, maxoutRate);
    lineTrace.run();

    // ライントレース後の走行距離を計算
    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // 走行距離が0であることをテスト
    EXPECT_EQ(0.0, endMileage - startMileage);
  }

  // マックスアウトの割合が範囲外の時、runでライントレースが実行されないことをテスト
  TEST_F(LineTraceTest, RunInvalidMaxoutRate)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    double targetSpeed = 100.0;       // 目標速度
    double targetDistance = 100.0;    // 目標距離
    int targetBrightness = 50;        // 目標とする明るさの値(%)
    double invalidMaxoutRate = -0.1;  // 範囲外のマックスアウトの割合

    // ライントレース前の走行距離を計算
    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // ライントレース動作を実行
    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, tracePid, deadbandRate, invalidMaxoutRate);
    lineTrace.run();

    // ライントレース後の走行距離を計算
    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // 走行距離が増加していないことをテスト
    EXPECT_EQ(0.0, endMileage - startMileage);
  }

  // デッドバンドの割合がマックスアウトの割合を上回る時、runでライントレースが実行されないことをテスト
  TEST_F(LineTraceTest, RunDeadbandRateGreaterThanMaxoutRate)
  {
    RealNetworkSystem netSys;
    SocketClient socketClient(netSys);
    Robot robot(socketClient);
    double targetSpeed = 100.0;      // 目標速度
    double targetDistance = 100.0;   // 目標距離
    int targetBrightness = 50;       // 目標とする明るさの値(%)
    double largeDeadbandRate = 0.5;  // マックスアウトの割合を上回るデッドバンドの割合
    double smallMaxoutRate = 0.3;    // デッドバンドの割合を下回るマックスアウトの割合

    // ライントレース前の走行距離を計算
    double startMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // ライントレース動作を実行
    LineTrace lineTrace(robot, std::make_unique<DistanceCondition>(robot, targetDistance),
                        targetSpeed, targetBrightness, tracePid, largeDeadbandRate,
                        smallMaxoutRate);
    lineTrace.run();

    // ライントレース後の走行距離を計算
    double endMileage
        = Mileage::calculateMileage(robot.getWheelMotorControllerInstance().getRightCount(),
                                    robot.getWheelMotorControllerInstance().getLeftCount());

    // 走行距離が増加していないことをテスト
    EXPECT_EQ(0.0, endMileage - startMileage);
  }
}  // namespace etrobocon2026_test