/**
 * @file ProjectedMileageTest.cpp
 * @brief ProjectedMileageクラスをテストする
 * @author miyahara046
 */

#include <gtest/gtest.h>
#include "ProjectedMileage.h"
#include "SystemInfo.h"
#include <cmath>
#include <limits>

namespace etrobocon2026_test {
  // 方位20度と-20度で走行した距離を基準方向に換算し、合計800mmになるかのテスト
  TEST(ProjectedMileageTest, TwentyDegreesAndRemainingStraightReach800)
  {
    ProjectedMileage mileage;
    mileage.reset(100.0, 0.0);
    mileage.update(100.0, 20.0);  // その場回頭
    mileage.update(600.0, 20.0);
    // 基準方向の移動距離は、500mm × cos(20度) = 約469.8463mmとなる。
    EXPECT_NEAR(mileage.getDistance(), 469.8463104, 1e-4);
    mileage.update(600.0, -20.0);
    // 基準方向の残り距離を、方位-20度で必要な走行距離に換算する。
    const double remaining = (800.0 - mileage.getDistance()) / std::cos(20.0 * PI / 180.0);
    mileage.update(600.0 + remaining, -20.0);
    EXPECT_NEAR(mileage.getDistance(), 800.0, 1e-4);
  }

  // 方位ごとの移動距離を積算し、後退で減算、リセットで0になるかのテスト
  TEST(ProjectedMileageTest, AccumulatesChangingHeadingsAndSubtractsReverse)
  {
    ProjectedMileage mileage;
    mileage.reset(0.0, 0.0);
    mileage.update(200.0, 0.0);
    mileage.update(200.0, 60.0);
    mileage.update(400.0, 60.0);
    // 200mm + 200mm × cos(60度) = 300mmとなる。
    EXPECT_NEAR(mileage.getDistance(), 300.0, 1e-4);
    mileage.update(300.0, 60.0);
    // 方位60度で100mm後退すると、基準方向の距離は50mm減る。
    EXPECT_NEAR(mileage.getDistance(), 250.0, 1e-4);
    mileage.reset(300.0, 0.0);
    EXPECT_DOUBLE_EQ(mileage.getDistance(), 0.0);
  }

  // 方位が179度から-179度へ変化したとき、中央方位を180度として計算するかのテスト
  TEST(ProjectedMileageTest, HeadingWrapUsesBackwardMidpoint)
  {
    ProjectedMileage mileage;
    mileage.reset(0.0, 179.0);
    mileage.update(100.0, -179.0);
    EXPECT_NEAR(mileage.getDistance(), -100.0, 1e-4);
  }

  // 不正な入力で距離を保持したまま無効になり、リセットで復帰するかのテスト
  TEST(ProjectedMileageTest, InvalidInputRequiresResetAndPreservesDistance)
  {
    ProjectedMileage mileage;
    EXPECT_FALSE(mileage.isValid());
    mileage.update(10.0, 0.0);
    EXPECT_FALSE(mileage.isValid());
    mileage.reset(0.0, 0.0);
    mileage.update(100.0, 0.0);
    // 方位がNaNの場合は、直前までの距離100mmを保持して計測を無効にする。
    mileage.update(200.0, std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(mileage.isValid());
    EXPECT_DOUBLE_EQ(mileage.getDistance(), 100.0);
    // 有限値で更新しても復帰せず、有効な起点でのリセットが必要となる。
    mileage.update(300.0, 0.0);
    EXPECT_FALSE(mileage.isValid());
    mileage.reset(300.0, 0.0);
    EXPECT_TRUE(mileage.isValid());
    EXPECT_DOUBLE_EQ(mileage.getDistance(), 0.0);
    // 起点の距離が無限大の場合も計測を無効にする。
    mileage.reset(std::numeric_limits<double>::infinity(), 0.0);
    EXPECT_FALSE(mileage.isValid());
  }
}  // namespace etrobocon2026_test
