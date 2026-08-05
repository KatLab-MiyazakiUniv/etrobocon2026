/**
 * @file   PositionTest.cpp
 * @brief  Positionクラスをテストする
 * @author yutaro-1214
 */

#include <gtest/gtest.h>

#include "Position.h"

namespace etrobocon2026_test {

  // Position生成直後に、X座標、Y座標、方位角が
  // すべて0で初期化されているかのテスト
  TEST(PositionTest, InitialValuesAreZero)
  {
    Position position;

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

  // set()を使用したとき、X座標、Y座標、方位角が
  // すべて指定した値に更新されるかのテスト
  TEST(PositionTest, SetAllValues)
  {
    Position position;

    double x = 100.0;
    double y = 200.0;
    double heading = 45.0;

    position.set(x, y, heading);

    double expectedX = 100.0;
    double expectedY = 200.0;
    double expectedHeading = 45.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // setX()を使用したとき、X座標だけが更新され、
  // Y座標と方位角が変化しないかのテスト
  TEST(PositionTest, SetX)
  {
    Position position;
    position.set(10.0, 20.0, 30.0);

    double x = 100.0;

    position.setX(x);

    double expectedX = 100.0;
    double expectedY = 20.0;
    double expectedHeading = 30.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // setY()を使用したとき、Y座標だけが更新され、
  // X座標と方位角が変化しないかのテスト
  TEST(PositionTest, SetY)
  {
    Position position;
    position.set(10.0, 20.0, 30.0);

    double y = 200.0;

    position.setY(y);

    double expectedX = 10.0;
    double expectedY = 200.0;
    double expectedHeading = 30.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // setHeading()を使用したとき、方位角だけが更新され、
  // X座標とY座標が変化しないかのテスト
  TEST(PositionTest, SetHeading)
  {
    Position position;
    position.set(10.0, 20.0, 30.0);

    double heading = 90.0;

    position.setHeading(heading);

    double expectedX = 10.0;
    double expectedY = 20.0;
    double expectedHeading = 90.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // 負の値を設定したとき、
  // X座標、Y座標、方位角が正しく保持されるかのテスト
  TEST(PositionTest, SetNegativeValues)
  {
    Position position;

    double x = -100.0;
    double y = -200.0;
    double heading = -90.0;

    position.set(x, y, heading);

    double expectedX = -100.0;
    double expectedY = -200.0;
    double expectedHeading = -90.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // 小数値を設定したとき、
  // X座標、Y座標、方位角が正しく保持されるかのテスト
  TEST(PositionTest, SetDecimalValues)
  {
    Position position;

    double x = 123.456;
    double y = -78.901;
    double heading = 45.678;

    position.set(x, y, heading);

    double expectedX = 123.456;
    double expectedY = -78.901;
    double expectedHeading = 45.678;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // set()を複数回実行したとき、
  // 最後に設定した値が保持されるかのテスト
  TEST(PositionTest, SetValuesMultipleTimes)
  {
    Position position;

    position.set(10.0, 20.0, 30.0);
    position.set(100.0, 200.0, 90.0);

    double expectedX = 100.0;
    double expectedY = 200.0;
    double expectedHeading = 90.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

  // 各セッターを順番に使用したとき、
  // それぞれの値が正しく更新されるかのテスト
  TEST(PositionTest, SetEachValueSeparately)
  {
    Position position;

    position.setX(100.0);
    position.setY(-200.0);
    position.setHeading(135.0);

    double expectedX = 100.0;
    double expectedY = -200.0;
    double expectedHeading = 135.0;

    double actualX = position.getX();
    double actualY = position.getY();
    double actualHeading = position.getHeading();

    EXPECT_DOUBLE_EQ(expectedX, actualX);
    EXPECT_DOUBLE_EQ(expectedY, actualY);
    EXPECT_DOUBLE_EQ(expectedHeading, actualHeading);
  }

}  // namespace etrobocon2026_test