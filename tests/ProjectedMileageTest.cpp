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
  TEST(ProjectedMileageTest, TwentyDegreesAndRemainingStraightReach800)
  {
    ProjectedMileage mileage;
    mileage.reset(100.0, 0.0);
    mileage.update(100.0, 20.0);  // その場回頭
    mileage.update(600.0, 20.0);
    EXPECT_NEAR(mileage.getDistance(), 469.8463104, 1e-4);
    mileage.update(600.0, -20.0);
    const double remaining = (800.0 - mileage.getDistance()) / std::cos(20.0 * PI / 180.0);
    mileage.update(600.0 + remaining, -20.0);
    EXPECT_NEAR(mileage.getDistance(), 800.0, 1e-4);
  }

  TEST(ProjectedMileageTest, AccumulatesChangingHeadingsAndSubtractsReverse)
  {
    ProjectedMileage mileage;
    mileage.reset(0.0, 0.0);
    mileage.update(200.0, 0.0);
    mileage.update(200.0, 60.0);
    mileage.update(400.0, 60.0);
    EXPECT_NEAR(mileage.getDistance(), 300.0, 1e-4);
    mileage.update(300.0, 60.0);
    EXPECT_NEAR(mileage.getDistance(), 250.0, 1e-4);
    mileage.reset(300.0, 0.0);
    EXPECT_DOUBLE_EQ(mileage.getDistance(), 0.0);
  }

  TEST(ProjectedMileageTest, HeadingWrapUsesBackwardMidpoint)
  {
    ProjectedMileage mileage;
    mileage.reset(0.0, 179.0);
    mileage.update(100.0, -179.0);
    EXPECT_NEAR(mileage.getDistance(), -100.0, 1e-4);
  }

}  // namespace etrobocon2026_test

namespace etrobocon2026_test {
  TEST(ProjectedMileageTest, InvalidInputRequiresResetAndPreservesDistance)
  {
    ProjectedMileage mileage;
    EXPECT_FALSE(mileage.isValid());
    mileage.update(10.0, 0.0);
    EXPECT_FALSE(mileage.isValid());
    mileage.reset(0.0, 0.0);
    mileage.update(100.0, 0.0);
    mileage.update(200.0, std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(mileage.isValid());
    EXPECT_DOUBLE_EQ(mileage.getDistance(), 100.0);
    mileage.update(300.0, 0.0);
    EXPECT_FALSE(mileage.isValid());
    mileage.reset(300.0, 0.0);
    EXPECT_TRUE(mileage.isValid());
    EXPECT_DOUBLE_EQ(mileage.getDistance(), 0.0);
    mileage.reset(std::numeric_limits<double>::infinity(), 0.0);
    EXPECT_FALSE(mileage.isValid());
  }
}  // namespace etrobocon2026_test
