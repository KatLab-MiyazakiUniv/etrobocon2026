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
    constexpr double ERROR_TOLERANCE = 1.0e-6;
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

    /*
     * 現在位置：(0, 0)
     * 目標地点：(3, 4)
     *
     * dx = 3 - 0 = 3
     * dy = 4 - 0 = 4
     *
     * distance = sqrt(dx^2 + dy^2)
     *          = sqrt(3^2 + 4^2)
     *          = sqrt(9 + 16)
     *          = 5
     */

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

    /*
     * 現在位置：(10, 20)
     * 目標地点：(13, 24)
     *
     * dx = 13 - 10 = 3
     * dy = 24 - 20 = 4
     *
     * distance = sqrt(3^2 + 4^2) = 5
     */

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

    /*
     * 現在位置：(10, 20)
     * 目標地点：(7, 16)
     *
     * dx = 7 - 10 = -3
     * dy = 16 - 20 = -4
     *
     * distance = sqrt((-3)^2 + (-4)^2)
     *          = sqrt(9 + 16)
     *          = 5
     */

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

    /*
     * dx = 100 - 100 = 0
     * dy = -200 - (-200) = 0
     *
     * distance = sqrt(0^2 + 0^2) = 0
     */

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

    /*
     * dx = 100
     * dy = 0
     *
     * heading = atan2(0, 100)
     *         = 0度
     */

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

    /*
     * dx = 0
     * dy = 100
     *
     * heading = atan2(100, 0)
     *         = 90度
     */

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

    /*
     * dx = 0
     * dy = -100
     *
     * heading = atan2(-100, 0)
     *         = -90度
     */

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

    /*
     * dx = -100
     * dy = 0
     *
     * heading = atan2(0, -100)
     *         = 180度
     *
     * AngleNormalizerの仕様によっては、
     * 180度が-180度へ正規化される可能性がある。
     */

    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_TRUE(std::abs(actual - 180.0) < ERROR_TOLERANCE
                || std::abs(actual + 180.0) < ERROR_TOLERANCE);
  }

  // 目標地点が右上方向にある場合、
  // 方位角が45度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingUpperRight)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = 100.0;
    double goalY = 100.0;

    /*
     * dx = 100
     * dy = 100
     *
     * heading = atan2(100, 100)
     *         = 45度
     */

    double expected = 45.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が左上方向にある場合、
  // 方位角が135度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingUpperLeft)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = -100.0;
    double goalY = 100.0;

    /*
     * dx = -100
     * dy = 100
     *
     * heading = atan2(100, -100)
     *         = 135度
     */

    double expected = 135.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が左下方向にある場合、
  // 方位角が-135度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingLowerLeft)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = -100.0;
    double goalY = -100.0;

    /*
     * dx = -100
     * dy = -100
     *
     * heading = atan2(-100, -100)
     *         = -135度
     */

    double expected = -135.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // 目標地点が右下方向にある場合、
  // 方位角が-45度になるかのテスト
  TEST(NavigatorTest, CalculateHeadingLowerRight)
  {
    Position position;
    position.set(0.0, 0.0, 0.0);

    Navigator navigator(position);

    double goalX = 100.0;
    double goalY = -100.0;

    /*
     * dx = 100
     * dy = -100
     *
     * heading = atan2(-100, 100)
     *         = -45度
     */

    double expected = -45.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
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

    /*
     * 現在位置：(100, 200)
     * 目標地点：(200, 300)
     *
     * dx = 200 - 100 = 100
     * dy = 300 - 200 = 100
     *
     * heading = atan2(100, 100)
     *         = 45度
     *
     * Positionに保存されている現在のheadingは、
     * calculateHeading()の結果には影響しない。
     */

    double expected = 45.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

  // Positionの座標を変更した後、
  // Navigatorが変更後の位置情報を使用するかのテスト
  TEST(NavigatorTest, UsesUpdatedPosition)
  {
    Position position;
    Navigator navigator(position);

    position.set(100.0, 100.0, 0.0);

    double goalX = 100.0;
    double goalY = 200.0;

    /*
     * NavigatorはPositionを参照として保持している。
     *
     * 変更後の現在位置：(100, 100)
     * 目標地点：(100, 200)
     *
     * dx = 0
     * dy = 100
     *
     * distance = 100
     * heading = 90度
     */

    double expectedDistance = 100.0;
    double expectedHeading = 90.0;

    double actualDistance = navigator.calculateDistance(goalX, goalY);
    double actualHeading = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expectedDistance, actualDistance, ERROR_TOLERANCE);
    EXPECT_NEAR(expectedHeading, actualHeading, ERROR_TOLERANCE);
  }

  // 現在位置と目標地点が同じ場合の方位角を確認するテスト
  TEST(NavigatorTest, CalculateHeadingSamePosition)
  {
    Position position;
    position.set(100.0, 200.0, 45.0);

    Navigator navigator(position);

    double goalX = 100.0;
    double goalY = 200.0;

    /*
     * dx = 0
     * dy = 0
     *
     * C++のstd::atan2(0, 0)は通常0を返すため、
     * 方位角は0度になる。
     *
     * 現在の実装では、Positionの現在方位角45度は返さない。
     */

    double expected = 0.0;
    double actual = navigator.calculateHeading(goalX, goalY);

    EXPECT_NEAR(expected, actual, ERROR_TOLERANCE);
  }

}  // namespace etrobocon2026_test