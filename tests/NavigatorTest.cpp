/**
 * @file   NavigatorTest.cpp
 * @brief  Navigatorクラスをテストする
 * @author yutaro-1214
 */

#include <cmath>
#include <gtest/gtest.h>

#include "Navigator.h"
#include "Position.h"

namespace etrobocon2026_test {

  namespace {
    constexpr double ERROR_TOLERANCE = 1.0e-5;
  }

  // 現在位置が原点、目標地点が(3, 4)のとき、
  // 目標地点までの距離が5になるかのテスト
  TEST(NavigatorTest, CalculateDistanceFromOrigin)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = 3.0;
    double goalY = 4.0;

    double expected = 5.0;
    double actual = navigator.calculateDistance(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 現在位置が原点以外の場合でも、
  // 目標地点までの距離が正しく計算されるかのテスト
  TEST(NavigatorTest, CalculateDistanceFromNonOriginPosition)
  {
    Position position;
    position.set(10.0, 20.0, 0.0);

    Navigator navigator(position);

    double goalX = 13.0;
    double goalY = 24.0;

    double expected = 5.0;
    double actual = navigator.calculateDistance(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が現在位置より左下にある場合でも、
  // 距離が正の値として計算されるかのテスト
  TEST(NavigatorTest, CalculateDistanceNegativeDirection)
  {
    Position position;
    position.set(10.0, 20.0, 0.0);

    Navigator navigator(position);

    double goalX = 7.0;
    double goalY = 16.0;

    double expected = 5.0;
    double actual = navigator.calculateDistance(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 現在位置と目標地点が同じ場合、
  // 距離が0になるかのテスト
  TEST(NavigatorTest, CalculateDistanceSamePosition)
  {
    Position position;
    position.set(100.0, -200.0, 30.0);

    Navigator navigator(position);

    double goalX = 100.0;
    double goalY = -200.0;

    double expected = 0.0;
    double actual = navigator.calculateDistance(goalX, goalY);

    EXPECT_DOUBLE_EQ(expected, actual);
  }

  // 目標地点が現在位置の右側にある場合、
  // 方位角が0度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingRight)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = 100.0;
    double goalY = 0.0;

    double expected = 0.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が現在位置の上側にある場合、
  // 方位角が90度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingUp)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = 0.0;
    double goalY = 100.0;

    double expected = 90.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が現在位置の下側にある場合、
  // 方位角が-90度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingDown)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = 0.0;
    double goalY = -100.0;

    double expected = -90.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が現在位置の左側にある場合、
  // 方位角が180度または-180度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingLeft)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = -100.0;
    double goalY = 0.0;

    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(180.0, std::abs(actual), ERROR_TOLERANCE);
  }

  // 現在位置が原点以外の場合でも、
  // 現在位置を基準に方位角が計算されるかのテスト
  TEST(NavigatorTest, CalculateHeadingFromNonOriginPosition)
  {
    Position position;
    position.set(100.0, 200.0, 30.0);

    Navigator navigator(position);

    double goalX = 200.0;
    double goalY = 300.0;

    double expected = 45.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 現在位置と目標地点が同じ場合の方位角を確認するテスト
  TEST(NavigatorTest, CalculateHeadingSamePosition)
  {
    Position position;
    position.set(100.0, 200.0, 45.0);

    Navigator navigator(position);

    double goalX = 100.0;
    double goalY = 200.0;

    double expected = 0.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

}  // namespace etrobocon2026_test