/**
 * @file   OdometryTest.cpp
 * @brief  Odometryクラスをテストする
 * @author yutaro-1214
 */

#include <cmath>
#include <gtest/gtest.h>

#include "Mileage.h"
#include "Odometry.h"
#include "Position.h"

namespace etrobocon2026_test {

  namespace {
    constexpr double ERROR_TOLERANCE = 1.0e-5;
  }

  // 初期状態で正のエンコーダ値を与え、方位角0度で更新したとき、
  // X座標だけが正方向に更新されるかのテスト
  TEST(OdometryTest, UpdateHeadingZero)
  {
    Position position;
    Odometry odometry(position);

    int32_t encoderLeft = 30;
    int32_t encoderRight = 40;
    double heading = 0.0;

    double distance = Mileage::calculateMileage(encoderRight, encoderLeft);
    double expectedX = distance;
    double expectedY = 0.0;
    double expectedHeading = 0.0;

    odometry.update(encoderLeft, encoderRight, heading);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // 方位角90度で更新したとき、
  // Y座標だけが正方向に更新されるかのテスト
  TEST(OdometryTest, UpdateHeadingNinety)
  {
    Position position;
    Odometry odometry(position);

    int32_t encoderLeft = 30;
    int32_t encoderRight = 40;
    double heading = 90.0;

    double distance = Mileage::calculateMileage(encoderRight, encoderLeft);
    double expectedX = 0.0;
    double expectedY = distance;
    double expectedHeading = 90.0;

    odometry.update(encoderLeft, encoderRight, heading);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // 方位角-90度で更新したとき、
  // Y座標が負方向に更新されるかのテスト
  TEST(OdometryTest, UpdateHeadingMinusNinety)
  {
    Position position;
    Odometry odometry(position);

    int32_t encoderLeft = 30;
    int32_t encoderRight = 40;
    double heading = -90.0;

    double distance = Mileage::calculateMileage(encoderRight, encoderLeft);
    double expectedX = 0.0;
    double expectedY = -distance;
    double expectedHeading = -90.0;

    odometry.update(encoderLeft, encoderRight, heading);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  //  負のエンコーダ差分を与えたとき、
  // ロボットの位置が後退方向に更新されるかのテスト
  TEST(OdometryTest, UpdateBackward)
  {
    Position position;
    Odometry odometry(position);

    int32_t encoderLeft = -30;
    int32_t encoderRight = -40;
    double heading = 0.0;

    double distance = Mileage::calculateMileage(encoderRight, encoderLeft);
    double expectedX = distance;
    double expectedY = 0.0;
    double expectedHeading = 0.0;

    odometry.update(encoderLeft, encoderRight, heading);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // initialize()で現在のエンコーダ値を保存した直後に、
  // 同じエンコーダ値でupdate()した場合、移動しないかのテスト
  TEST(OdometryTest, InitializePreventsInitialMovement)
  {
    Position position;
    Odometry odometry(position);

    int32_t encoderLeft = 1000;
    int32_t encoderRight = 1200;
    double heading = 45.0;

    odometry.initialize(encoderLeft, encoderRight);
    odometry.update(encoderLeft, encoderRight, heading);

    double expectedX = 0.0;
    double expectedY = 0.0;
    double expectedHeading = 45.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // initialize()後にエンコーダ値が増加したとき、
  // 初期値からの差分だけで位置が更新されるかのテスト
  TEST(OdometryTest, UpdateUsesEncoderDifferenceAfterInitialize)
  {
    Position position;
    Odometry odometry(position);

    int32_t initialEncoderLeft = 1000;
    int32_t initialEncoderRight = 1200;

    int32_t currentEncoderLeft = 1030;
    int32_t currentEncoderRight = 1240;

    double heading = 0.0;

    int32_t dEncoderLeft = currentEncoderLeft - initialEncoderLeft;
    int32_t dEncoderRight = currentEncoderRight - initialEncoderRight;

    double distance = Mileage::calculateMileage(dEncoderRight, dEncoderLeft);
    double expectedX = distance;
    double expectedY = 0.0;
    double expectedHeading = 0.0;

    odometry.initialize(initialEncoderLeft, initialEncoderRight);
    odometry.update(currentEncoderLeft, currentEncoderRight, heading);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // update()を複数回実行したとき、
  // 各回の移動量がPositionへ累積されるかのテスト
  TEST(OdometryTest, UpdateAccumulatesPosition)
  {
    Position position;
    Odometry odometry(position);

    double firstDistance = Mileage::calculateMileage(360, 360);
    double secondDistance = Mileage::calculateMileage(360, 360);

    double expectedX = firstDistance;
    double expectedY = secondDistance;
    double expectedHeading = 90.0;

    odometry.update(360, 360, 0.0);
    odometry.update(720, 720, 90.0);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // 360度を超える方位角が正規化されてから
  // Positionに保存されるかのテスト
  TEST(OdometryTest, NormalizeHeadingOverPositiveRange)
  {
    Position position;
    Odometry odometry(position);

    int32_t encoderLeft = 30;
    int32_t encoderRight = 40;
    double heading = 450.0;

    double distance = Mileage::calculateMileage(encoderRight, encoderLeft);
    double expectedX = 0.0;
    double expectedY = distance;
    double expectedHeading = 90.0;

    odometry.update(encoderLeft, encoderRight, heading);

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_NEAR(expectedX, actualX, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedY, actualY, ERROR_TOLERANCE);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // reset()を実行したとき、
  // PositionのX座標、Y座標、方位角が0に戻るかのテスト
  TEST(OdometryTest, ResetPosition)
  {
    Position position;
    Odometry odometry(position);

    position.set(100.0, -200.0, 45.0);

    odometry.reset();

    double expectedX = 0.0;
    double expectedY = 0.0;
    double expectedHeading = 0.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

}  // namespace etrobocon2026_test